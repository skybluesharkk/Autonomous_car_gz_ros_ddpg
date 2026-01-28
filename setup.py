from setuptools import setup
import os
from glob import glob

package_name = 'gazebo_car_sim_package'
ddpg_submodule = 'ddpg_algorithm'

def package_files(data_files, directory_list):
    paths_dict = {}
    for (path, directories, filenames) in os.walk(directory_list):
        for filename in filenames:
            file_path = os.path.join(path, filename)
            install_path = os.path.join('share', package_name, path)
            if install_path in paths_dict.keys():
                paths_dict[install_path].append(file_path)
            else:
                paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'ddpg_algorithm'), glob('ddpg_algorithm/*.py')),
]

package_files(data_files, 'models')
package_files(data_files, 'worlds')

setup(
    name=package_name,
    version='0.0.0',
    packages=[ddpg_submodule],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David',
    maintainer_email='david@example.com',
    description='Simulated car environment for DDPG training',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'env_node = ddpg_algorithm.env_node:main',
            'train = ddpg_algorithm.train:main',
            'service_node = ddpg_algorithm.service_node:main',
        ],
    },
)
