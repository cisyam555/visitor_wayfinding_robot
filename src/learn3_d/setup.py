from setuptools import find_packages, setup

package_name = 'learn3_d'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/places.yaml']),
        ('share/' + package_name + '/launch', ['launch/tour.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='消防地图语音导览：语音/文本 -> 千问理解 -> 定点导航 + TTS 播报',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'chat_say = learn3_d.chat_say:main',
            'nav_route = learn3_d.nav_route:main',
            'tour_chat = learn3_d.tour_chat:main',
        ],
    },
)
