from setuptools import setup

package_name = 'lab5_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzangupenn',
    maintainer_email='zzang@seas.upenn.edu',
    description='f1tenth lab5_pkg',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scanmatch_node = lab5_pkg.scanmatch_node:main',
        ],
    },
)
