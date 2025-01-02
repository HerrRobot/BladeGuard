from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='urosgluscevic',
    maintainer_email='U.Gluscevic@student.tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_executor=motors.serial_executor:main',
            'test_sensors=motors.UI_testing:main',
            'service_test=motors.service_test:main'
        ],
    },
)
