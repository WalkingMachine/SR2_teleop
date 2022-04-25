import os
from setuptools import setup
from glob import glob

package_name = 'sr2_teleop'
xbox_controller = 'sr2_teleop/xbox_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.yaml'))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='walking1',
    maintainer_email='nimai.jariwala.1@ens.etsmtl.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "start_reading = sr2_teleop.teleop:main",
            "start_control = sr2_teleop.teleop_control:main"
        ],
    },
)
