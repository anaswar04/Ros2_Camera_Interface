from setuptools import setup

package_name = 'camera_switcher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
     ['resource/camera_switcher']),
    ('share/camera_switcher', ['package.xml']),
    ('share/camera_switcher/launch', ['launch/camera_switcher.launch.py']),
    ('share/camera_switcher', ['camera_params.yaml']),
],


    install_requires=[
        'setuptools',
        'opencv-python'  # We'll use OpenCV for camera access and image display
    ],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='ROS2 package to publish one camera feed at a time and display it',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_switcher.camera_publisher:main',
            'camera_subscriber = camera_switcher.camera_subscriber:main',
        ],
    },
)
