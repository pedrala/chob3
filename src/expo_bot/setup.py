from setuptools import find_packages, setup

package_name = 'expo_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='viator',
    maintainer_email='viator@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cctv_detection_node = expo_bot.cctv_detection_node:main',
            'navigate_to_goal_node = expo_bot.navigate_to_goal_node:main',
            'amr_control_node = expo_bot.amr_control_node:main',
            'object_detection_node = expo_bot.object_detection_node:main',
            'video_playback_node = expo_bot.video_playback_node:main',
            'sys_monitor_node = expo_bot.sys_monitor_node:main',
        ],
    },

)
