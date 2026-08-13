from setuptools import setup

package_name = 'bag_replay_pkg'


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bag_lane_replay.launch.py']),
        ('share/' + package_name + '/models', ['models/best.pt']),
        ('share/' + package_name + '/bags', ['bags/README.md', 'bags/.gitignore']),
        ('share/' + package_name, ['README.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenAI Codex',
    maintainer_email='support@openai.com',
    description='Replay recorded ROS 2 bags and rerun lane detection with a packaged YOLO model.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
