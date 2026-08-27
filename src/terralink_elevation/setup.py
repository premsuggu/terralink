from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'terralink_elevation'

# Paths relative to this setup.py file
package_root = os.path.dirname(os.path.abspath(__file__))

# Get relative paths for data_files
def get_data_files():
    data_files = []
    
    # Launch files
    launch_files = glob(os.path.join(package_root, 'launch', '*.launch.py'))
    if launch_files:
        data_files.append((os.path.join('share', package_name, 'launch'), 
                          [os.path.relpath(f, package_root) for f in launch_files]))
    
    # Config files
    config_files = glob(os.path.join(package_root, 'config', '*.yaml'))
    if config_files:
        data_files.append((os.path.join('share', package_name, 'config'), 
                          [os.path.relpath(f, package_root) for f in config_files]))
    
    # RViz files
    rviz_files = glob(os.path.join(package_root, 'rviz', '*.rviz'))
    if rviz_files:
        data_files.append((os.path.join('share', package_name, 'rviz'), 
                          [os.path.relpath(f, package_root) for f in rviz_files]))
    
    # World files
    world_files = glob(os.path.join(package_root, 'worlds', '*.world'))
    if world_files:
        data_files.append((os.path.join('share', package_name, 'worlds'), 
                          [os.path.relpath(f, package_root) for f in world_files]))
    
    # Description files (SDF/URDF)
    description_files = glob(os.path.join(package_root, 'description', '*.sdf'))
    description_files += glob(os.path.join(package_root, 'description', '*.urdf'))
    description_files += glob(os.path.join(package_root, 'description', '*.xacro'))
    if description_files:
        data_files.append((os.path.join('share', package_name, 'description'), 
                          [os.path.relpath(f, package_root) for f in description_files]))
    
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + get_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prem',
    maintainer_email='prem@terralink.local',
    description='UAV Elevation Mapping for TerraLink - 2.5D GPU-accelerated elevation mapping',
    license='MIT',
    tests_require=['pytest'],
    scripts=[
        'scripts/synthetic_pointcloud.py',
        'scripts/synthetic_pointcloud_tf_publisher.py',
        'scripts/elevation_mapping_node.py',
    ],
)