import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'emap'
package_root = os.path.dirname(os.path.abspath(__file__))


def files_by_dir(*rel_dirs):
    """(share_subpath, [files]) pairs for every file under each rel_dir,
    preserving nested subdirectories (needed for models/iris_quad/meshes/)."""
    data_files = []
    for rel_dir in rel_dirs:
        root = os.path.join(package_root, rel_dir)
        for dirpath, _, filenames in os.walk(root):
            if not filenames:
                continue
            share_subpath = os.path.join('share', package_name, os.path.relpath(dirpath, package_root))
            data_files.append((share_subpath, [os.path.relpath(os.path.join(dirpath, f), package_root) for f in filenames]))
    return data_files


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + files_by_dir('launch', 'worlds', 'config', 'models', 'rviz'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TerraLink emap',
    maintainer_email='software.guild@smail.iitm.ac.in',
    description='TerraLink emap - from-scratch UAV elevation mapping, rebuilt on Ignition Gazebo Fortress + ros_gz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'elevation_mapping_node = emap.elevation_mapping_node:main',
            'cmd_vel_watchdog = emap.cmd_vel_watchdog:main',
        ],
    },
)
