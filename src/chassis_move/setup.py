# 从 `setuptools` 引入包发现和安装入口。
from setuptools import find_packages, setup

# 统一定义包名，避免后面多处硬编码。
package_name = 'chassis_move'

# 调用 setuptools 安装配置入口。
setup(
    # Python 包名称。
    name=package_name,
    # 当前示例包版本号。
    version='0.0.0',
    # 自动查找要安装的 Python 包，排除测试目录。
    packages=find_packages(exclude=['test']),
    # 安装时一并拷贝的数据文件列表。
    data_files=[
        # 注册到 ament 包索引中。
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 安装 ROS 包元数据文件。
        ('share/' + package_name, ['package.xml']),
        (
            # 安装 launch 文件到共享目录。
            'share/' + package_name + '/launch',
            [
                'launch/protocol_bringup.launch.py',
                'launch/protocol_command_bridge.launch.py',
                'launch/protocol_airmouse.launch.py',
                'launch/protocol_airmouse_bringup.launch.py',
                'launch/robot_status_pose_bridge.launch.py',
            ],
        ),
        (
            # 安装 YAML 配置文件到共享目录。
            'share/' + package_name + '/config',
            [
                'config/protocol_params.yaml',
            ],
        ),
    ],
    # 运行时基础依赖。
    install_requires=['setuptools'],
    # 允许以 zip 形式安全分发。
    zip_safe=True,
    # 维护者名称。
    maintainer='raybot',
    # 维护者邮箱。
    maintainer_email='rayruimincheng@gmail.com',
    # 包功能简介。
    description='ROS 2 scheduler protocol tools and callback gateway for Orange Pi based robots',
    # 开源许可证。
    license='Apache-2.0',
    # 测试相关的额外依赖集合。
    extras_require={
        'test': [
            'pytest',
        ],
    },
    # 控制台脚本入口定义。
    entry_points={
        'console_scripts': [
            # Protocol path: scheduler-style HTTP control and callbacks.
            'protocol_gateway = chassis_move.protocol_gateway_node:main',
            'protocol_command_bridge = chassis_move.protocol_command_bridge_node:main',
            'protocol_teleop = chassis_move.protocol_teleop_node:main',
            'protocol_airmouse = chassis_move.protocol_airmouse_node:main',
            'protocol_cli = chassis_move.protocol_cli:main',
            'robot_status_pose_bridge = chassis_move.robot_status_pose_bridge_node:main',
        ],
    },
)
