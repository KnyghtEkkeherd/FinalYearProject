from setuptools import find_packages, setup

package_name = 'medicine_dispenser'

setup(
    name=package_name,
    version='0.0.0',
    packages=['medicine_dispenser'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gyattbot',
    maintainer_email='wmkowalczyk@connect.ust.hk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dispenser=medicine_dispenser.dispenser:main'
        ],
    },
)
