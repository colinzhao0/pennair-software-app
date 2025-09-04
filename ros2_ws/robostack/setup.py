from setuptools import setup

package_name = 'robostack'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='ROS 2 package for shape detection and video streaming nodes.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_streamer_node = robostack.video_streamer_node:main',
            'shape_detection_node = robostack.shape_detection_node:main',
        ],
    },
)
