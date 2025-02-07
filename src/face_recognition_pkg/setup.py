from setuptools import setup

package_name = 'face_recognition_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Include package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'face_recognition', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='ARMAAN DAYAL',
    maintainer_email='adayal@connect.ust.hk',
    description='Face recognition ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'facial_recognition_node = face_recognition_pkg.face_recognition_node:main',
        ],
    },
)
