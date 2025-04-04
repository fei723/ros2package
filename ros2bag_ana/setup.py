from setuptools import setup
from glob import glob
from setuptools import find_packages
import os
package_name = 'ros2bag_ana'  

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),  # 推荐使用 os.path.join
        glob('config/*.yaml')),  # 推荐使用 glob 模式匹配多个配置文件
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shifei',
    maintainer_email='shifei3@xiaomi.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'total_dynamic = ros2bag_ana.total_dynamic:main',  
        ],
    },
    # install_requires=[
    #     'numpy',
    #     'tqdm',
    #     'watchdog',
    # ],
)

