from setuptools import setup
import os
from glob import glob

package_name = 'stanley_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'racelines'), glob('racelines/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Steven Gong',
    maintainer_email='s36gong@uwaterloo.ca',
    description='stanley avoidance',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stanley_avoidance = stanley_avoidance.stanley_avoidance:main',
        ],
    },
)
