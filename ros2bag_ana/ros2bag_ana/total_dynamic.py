import os
import csv
import numpy as np
import json
import datetime
from collections import defaultdict
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import argparse
import ast
import tqdm
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import threading
import sys
import time
import re


processed_files = set()
observer = None


class BagFileHandler(FileSystemEventHandler):
    """监控文件夹变化的事件处理器"""
    def __init__(self, input_folder, output_dir, tf_joints, angle_joints_regex):
        self.input_folder = input_folder
        self.output_dir = output_dir
        self.tf_joints = tf_joints
        self.angle_joints_regex = angle_joints_regex

    def on_created(self, event):
        """当新文件创建时触发，但延迟处理直到文件写入完成"""
        global processed_files
        # 原有代码
        if not event.is_directory and event.src_path.endswith('.mcap'):
            file_path = event.src_path
            filename = os.path.basename(file_path)
            if filename not in processed_files:
                print(f"\n检测到新文件：{filename}，开始监控文件写入完成...")
                processed_files.add(filename)  # 避免重复处理
                threading.Thread(
                    target=self.wait_for_file_completion,
                    args=(file_path, filename)
                ).start()

    def wait_for_file_completion(self, file_path, filename):
        """等待文件写入完成"""
        timeout_seconds = 15  # 最大等待时间（秒）
        check_interval = 1    # 检查间隔（秒）
        previous_size = 0
        stable_duration = 3   # 连续稳定时间（秒）

        start_time = time.time()
        while True:
            current_size = os.path.getsize(file_path)
            if current_size == previous_size:
                # 文件大小稳定
                if time.time() - start_time >= stable_duration:
                    break
            else:
                previous_size = current_size
                start_time = time.time()

            time.sleep(check_interval)
            if time.time() - start_time > timeout_seconds:
                print(f"警告：文件 {filename} 超过等待时间，可能未完成写入，但继续处理...")
                break

        # 处理文件
        print(f"\n检测到文件 {filename} 写入完成，开始处理...")
        self.process_file(file_path)

    def process_file(self, bag_path):
        """处理单个.mcap文件并更新CSV"""
        try:
            result = process_bag_file(
                bag_path,
                self.tf_joints,
                self.angle_joints_regex
            )
            
            # 更新中间文件（需加锁避免并发问题）
            with open(intermediate_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    os.path.basename(bag_path),
                    result["duration"],
                    json.dumps(result["tf_distances"]),
                    json.dumps(result["tf_displacements"]),
                    result["left_angle_total"],
                    result["right_angle_total"]
                ])
            
            # 重新计算评分并更新最终文件
            score_csv(intermediate_file, final_output_file)
            print(f"处理完成：{os.path.basename(bag_path)}")
        except Exception as e:
            print(f"错误：处理 {bag_path} 时出错：{str(e)}")

def process_bag_file(bag_path, tf_joints, angle_joints_regex, topics_to_read=['/tf', '/joint_states']):
    start_time = None
    end_time = None

    tf_distances = {joint: 0.0 for joint in tf_joints}
    tf_prev_pos = {joint: None for joint in tf_joints}
    tf_initial_pos = {joint: None for joint in tf_joints}

    angle_changes = defaultdict(float)
    angle_prev = {}

    storage_options = StorageOptions(
        uri=bag_path,
        storage_id='mcap',
    )
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    while reader.has_next():
        try:
            topic, data, t = reader.read_next()
            if topic not in topics_to_read:
                continue

            if topic == '/tf':
                msg_type = get_message('tf2_msgs/msg/TFMessage')
                tf_msg = deserialize_message(data, msg_type)

                for transform in tf_msg.transforms:
                    child_frame = transform.child_frame_id

                    if child_frame in tf_joints:
                        pos = np.array([
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z
                        ], dtype=np.float32)

                        if start_time is None:
                            start_time = t
                        end_time = t

                        prev_pos = tf_prev_pos[child_frame]
                        initial_pos = tf_initial_pos[child_frame]

                        if prev_pos is not None:
                            delta = pos - prev_pos
                            distance = float(np.linalg.norm(delta))
                            tf_distances[child_frame] += distance

                        tf_prev_pos[child_frame] = pos.copy()

                        if initial_pos is None:
                            tf_initial_pos[child_frame] = pos.copy()

            elif topic == '/joint_states':
                msg_type = get_message('sensor_msgs/msg/JointState')
                joint_state_msg = deserialize_message(data, msg_type)

                for name, position in zip(joint_state_msg.name, joint_state_msg.position):
                    if re.search(angle_joints_regex, name):
                        current_angle = position

                        if name not in angle_prev:
                            angle_prev[name] = None

                        if angle_prev[name] is not None:
                            angle_changes[name] += abs(current_angle - angle_prev[name])

                        angle_prev[name] = current_angle

        except Exception as e:
            print(f"Error processing message: {e}")

    duration = (end_time - start_time) / 1e9 if start_time and end_time else 0.0

    tf_displacements = {}
    for joint in tf_joints:
        if (tf_initial_pos[joint] is not None and 
            tf_prev_pos[joint] is not None):
            displacement = float(np.linalg.norm(
                tf_prev_pos[joint] - tf_initial_pos[joint]
            ))
            tf_displacements[joint] = displacement
        else:
            tf_displacements[joint] = 0.0

    left_angle = sum(v for k, v in angle_changes.items() if k.startswith('l_'))
    right_angle = sum(v for k, v in angle_changes.items() if k.startswith('r_'))

    return {
        "duration": duration,
        "tf_distances": tf_distances,
        "tf_displacements": tf_displacements,
        "left_angle_total": left_angle,
        "right_angle_total": right_angle
    }

def generate_csv(input_folder, output_file, tf_joints, angle_joints_regex, topics_to_read=['/tf', '/joint_states']):
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "Bag File", "Duration",
            "TF Distances", "TF Displacements",
            "Left_Angle_Total", "Right_Angle_Total"
        ])

        # 收集所有.mcap文件路径
        all_files = []
        for root, _, files in os.walk(input_folder):
            for filename in files:
                if filename.endswith('.mcap'):
                    all_files.append(os.path.join(root, filename))
        
        total_files = len(all_files)
        if total_files == 0:
            print("错误：未找到任何 .mcap 文件！")
            return
        
        # 初始化进度条
        pbar = tqdm.tqdm(
            total=total_files,
            desc=f"处理中 (0/{total_files})",
            unit="文件",
            dynamic_ncols=True,
            colour='green'
        )

        for idx, file_path in enumerate(all_files):
            filename = os.path.basename(file_path)
            try:
                result = process_bag_file(
                    file_path,
                    tf_joints,
                    angle_joints_regex,
                    topics_to_read
                )
                writer.writerow([
                    filename,
                    result["duration"],
                    json.dumps(result["tf_distances"]),
                    json.dumps(result["tf_displacements"]),
                    result["left_angle_total"],
                    result["right_angle_total"]
                ])
            except Exception as e:
                tqdm.tqdm.write(f"错误：处理 {filename} 时出错：{str(e)}")
            
            # 更新进度条描述
            pbar.set_description(f"处理中 ({idx+1}/{total_files})")
            pbar.update(1)
        
        pbar.close()
def normalize(value, min_val, max_val):
    if max_val == min_val:
        return 2.5  
    return (value - min_val) / (max_val - min_val) * 5  

def calculate_score(row_data):
    return (
        0.7 * row_data['Duration_Norm'] +
        0.2 * row_data['End_Distance_Norm'] +
        0.1 * row_data['Angle_Total_Norm']
    )

def score_csv(input_file, output_file):
    data = []
    with open(input_file, 'r') as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames.copy()  
        
        new_columns = [
            'end_distance', 'Angle_Total',
            'Duration_Norm', 'End_Distance_Norm',
            'Angle_Total_Norm', 'Score', 'Percentage'
        ]
        fieldnames.extend(new_columns)  
        
        for row in reader:
            entry = row.copy()
            
            try:
                tf_dict = ast.literal_eval(entry['TF Distances'])
                r_dist = tf_dict.get('r_wrist_camera2_color_mujoco_frame', 0)
                l_dist = tf_dict.get('l_wrist_camera2_color_mujoco_frame', 0)
                entry['end_distance'] = r_dist + l_dist
            except:
                entry['end_distance'] = 0.0
            
            entry['Angle_Total'] = float(entry['Left_Angle_Total']) + float(entry['Right_Angle_Total'])
            
            entry['Duration'] = float(entry['Duration'])
            entry['Left_Angle_Total'] = float(entry['Left_Angle_Total'])
            entry['Right_Angle_Total'] = float(entry['Right_Angle_Total'])
            
            data.append(entry)
    
    durations = [d['Duration'] for d in data]
    end_distances = [d['end_distance'] for d in data]
    angle_totals = [d['Angle_Total'] for d in data]
    
    min_duration, max_duration = min(durations), max(durations)
    min_end, max_end = min(end_distances), max(end_distances)
    min_angle, max_angle = min(angle_totals), max(angle_totals)
    
    for entry in data:
        entry['Duration_Norm'] = round(normalize(entry['Duration'], min_duration, max_duration), 2)
        entry['End_Distance_Norm'] = round(normalize(entry['end_distance'], min_end, max_end), 2)
        entry['Angle_Total_Norm'] = round(normalize(entry['Angle_Total'], min_angle, max_angle), 2)
        entry['Score'] = round(calculate_score(entry), 2)
    
    scores = [d['Score'] for d in data]
    min_score, max_score = min(scores), max(scores)
    
    for entry in data:
        score = entry['Score']
        if max_score == min_score:
            percentage = 80.0
        else:
            percentage = 60.0 + (40.0 * ( (max_score - score) / (max_score - min_score) ))
        entry['Percentage'] = round(percentage, 2)
    
    with open(output_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(data)

def main():
    print("ROS Bag处理工具 - 交互式参数设置")
    
    # 选择模式
    while True:
        print("\n请选择运行模式：")
        print("1. 处理现有文件（单次运行）")
        print("2. 启动实时监控模式（持续监听文件夹）")
        mode = input("输入选项（1/2）：")
        if mode in ['1', '2']:
            break
        else:
            print("错误：请输入 1 或 2！")
    
    # 输入路径
    while True:
        input_folder = input("请输入包含.mcap文件的输入文件夹路径(支持2层嵌套): ")
        if os.path.exists(input_folder):
            break
        else:
            print("错误：路径不存在，请重新输入！")
    
    # 输出路径
    output_dir = input("请输入输出结果的目录路径（自动生成文件名）: ")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # 设置tf_joints参数
    tf_joints = ['r_wrist_camera2_color_mujoco_frame', 'l_wrist_camera2_color_mujoco_frame']
    modify_tf = input(f"当前tf_joints参数为：{tf_joints}\n是否修改？(Y/N): ").lower()
    if modify_tf == 'y':
        new_joints = input("请输入新的tf_joints（用逗号分隔）：").split(',')
        tf_joints = [joint.strip() for joint in new_joints]
    
    # 设置angle_joints_regex参数
    angle_joints_regex = r'_arm'
    modify_regex = input(f"当前angle_joints正则表达式为：{angle_joints_regex}\n是否修改？(Y/N): ").lower()
    if modify_regex == 'y':
        angle_joints_regex = input("请输入新的正则表达式：")
    
    # 时间戳生成
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 生成中间文件路径
    global intermediate_file
    intermediate_file = os.path.join(
        output_dir,
        f"intermediate_{timestamp}.csv"
    )
    
    # 最终文件路径
    global final_output_file
    final_output_file = os.path.join(
        output_dir,
        f"final_scores_{timestamp}.csv"
    )
    
    if mode == '1':
        # 模式1：处理现有文件
        print("\n正在生成原始数据CSV...")
        generate_csv(
            input_folder,
            intermediate_file,
            tf_joints,
            angle_joints_regex
        )
        
        print("正在计算评分并生成最终CSV...")
        score_csv(intermediate_file, final_output_file)
        
        os.remove(intermediate_file)
        print(f"\n处理完成！\n最终结果保存在：{final_output_file}")
    
    elif mode == '2':
        # 模式2：实时监控模式
        print("\n已进入实时监控模式，按 'q' 回车键可退出")
        print(f"监控目录：{input_folder}")
        print(f"输出目录：{output_dir}")
        
        # 初始化中间文件
        with open(intermediate_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "Bag File", "Duration",
                "TF Distances", "TF Displacements",
                "Left_Angle_Total", "Right_Angle_Total"
            ])
        
        # 启动监控
        event_handler = BagFileHandler(
            input_folder,
            output_dir,
            tf_joints,
            angle_joints_regex
        )
        global observer
        observer = Observer()
        observer.schedule(event_handler, input_folder, recursive=True)
        observer.start()
        
        # 新增：通过输入 'q' 停止监控
        quit_flag = False
        
        def input_thread_func():
            nonlocal quit_flag
            input("按 'q' 然后回车键停止监控: ")
            quit_flag = True
        
        input_thread = threading.Thread(target=input_thread_func)
        input_thread.daemon = True
        input_thread.start()
        
        try:
            while not quit_flag:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n检测到 Ctrl+C，正在退出...")
        
        # 停止监控
        observer.stop()
        observer.join()
        
        # 清理
        score_csv(intermediate_file, final_output_file)
        os.remove(intermediate_file)
        print(f"\n监控已停止，最终结果保存在：{final_output_file}")

if __name__ == "__main__":
    # 安装必要依赖（如果未安装）
    try:
        import watchdog
    except ImportError:
        print("检测到缺少依赖库，正在安装...")
        os.system("pip install watchdog")
    
    main()