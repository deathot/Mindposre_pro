from setuptools import setup
from glob import glob
import os

package_name = 'service_client'
pkg_utils_path = package_name + ".utils"
pkg_acllite_path = package_name + ".acllite"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, pkg_utils_path, pkg_acllite_path],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "service_client = service_client.client:main",
            "service_server = service_client.server:main"
        ],
    },
)
