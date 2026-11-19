"""一键启动消防地图语音导览。

用法：
    source /opt/ros/jazzy/setup.bash
    source ~/xos/setup.bash
    source ~/guide_robot/learn/visitor_wayfinding_robot/install/setup.bash
    ros2 launch learn3_d tour.launch.py

换地图或点位不用改文件，命令行覆盖即可：
    ros2 launch learn3_d tour.launch.py map:=消防 reloc_poi:=乐知教育
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('map', default_value='消防',
                              description='底盘地图名'),
        DeclareLaunchArgument('reloc_poi', default_value='乐知教育',
                              description='重定位参考点位（也是待命点）'),
        DeclareLaunchArgument('track', default_value='',
                              description='轨道名；空字符串走点对点（推荐单点带路），'
                                          '显式填写才按录制轨道走'),
        DeclareLaunchArgument('enable_nav', default_value='true',
                              description='是否连接底盘做导航'),
        DeclareLaunchArgument('nav_direct', default_value='true',
                              description='true=直接前往目标点；false=按主线逐段走（每站会停）'),
        DeclareLaunchArgument('wake_word', default_value='小静',
                              description='语音唤醒词，启动时自动登记到平台'),
        DeclareLaunchArgument('enable_voice', default_value='true',
                              description='是否使用平台 TTS 播报'),
        DeclareLaunchArgument('enable_greet', default_value='false',
                              description='开机是否立刻打招呼（默认关，改为有人靠近再说）'),
        DeclareLaunchArgument('enable_presence', default_value='true',
                              description='是否用头部深度图检测有人靠近并主动问候'),
        DeclareLaunchArgument('presence_distance', default_value='0.8',
                              description='触发问候的前方距离（米）'),
        DeclareLaunchArgument('await_timeout', default_value='35.0',
                              description='送达后多少秒没人搭话就自动回待命点，0 表示关闭'),
        DeclareLaunchArgument('pause_timeout', default_value='35.0',
                              description='导航中被叫停后多少秒没人再说话就自动回待命点，0 表示关闭'),
        DeclareLaunchArgument('pending_timeout', default_value='90.0',
                              description='提议目的地后多少秒内说「好的」才算确认，0 表示不过期'),
        DeclareLaunchArgument('qwen_base_url',
                              default_value='http://120.48.75.178:7006/v1',
                              description='千问 OpenAI 兼容接口地址'),
        DeclareLaunchArgument('qwen_model', default_value='Qwen3.5-122B-A10B',
                              description='模型名'),
        DeclareLaunchArgument('qwen_timeout', default_value='30.0',
                              description='模型请求超时（秒）'),
        DeclareLaunchArgument('asr_model', default_value='small',
                              description='本地识别模型：small 更快，medium 更准（需先下载）'),
        DeclareLaunchArgument('asr_beam_size', default_value='3',
                              description='识别 beam，越大越准越慢'),
        DeclareLaunchArgument('vad_threshold', default_value='300.0',
                              description='能量 VAD 阈值，越小越灵敏（嘈杂环境调大）'),
        DeclareLaunchArgument('vad_silence_s', default_value='0.4',
                              description='多久没声音算一句话结束（秒）'),
    ]

    tour_chat = Node(
        package='learn3_d',
        executable='tour_chat',
        name='tour_chat',
        output='screen',
        parameters=[{
            'map': LaunchConfiguration('map'),
            'reloc_poi': LaunchConfiguration('reloc_poi'),
            'track': LaunchConfiguration('track'),
            'nav_direct': ParameterValue(
                LaunchConfiguration('nav_direct'), value_type=bool),
            'enable_nav': ParameterValue(
                LaunchConfiguration('enable_nav'), value_type=bool),
            'wake_word': LaunchConfiguration('wake_word'),
            'enable_voice': ParameterValue(
                LaunchConfiguration('enable_voice'), value_type=bool),
            'enable_greet': ParameterValue(
                LaunchConfiguration('enable_greet'), value_type=bool),
            'enable_presence': ParameterValue(
                LaunchConfiguration('enable_presence'), value_type=bool),
            'presence_distance': ParameterValue(
                LaunchConfiguration('presence_distance'), value_type=float),
            'await_timeout': ParameterValue(
                LaunchConfiguration('await_timeout'), value_type=float),
            'pause_timeout': ParameterValue(
                LaunchConfiguration('pause_timeout'), value_type=float),
            'pending_timeout': ParameterValue(
                LaunchConfiguration('pending_timeout'), value_type=float),
            'qwen_base_url': LaunchConfiguration('qwen_base_url'),
            'qwen_model': LaunchConfiguration('qwen_model'),
            'qwen_timeout': ParameterValue(
                LaunchConfiguration('qwen_timeout'), value_type=float),
            'asr_model': LaunchConfiguration('asr_model'),
            'asr_beam_size': ParameterValue(
                LaunchConfiguration('asr_beam_size'), value_type=int),
            'vad_threshold': ParameterValue(
                LaunchConfiguration('vad_threshold'), value_type=float),
            'vad_silence_s': ParameterValue(
                LaunchConfiguration('vad_silence_s'), value_type=float),
        }],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='启动消防地图语音导览（底盘导航 + 千问对话 + TTS）'),
        tour_chat,
    ])
