from setuptools import setup, find_packages

package_name = 'simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    description='Simulation package with a Python Webots controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'simulation = controllers.simulation:main'
        ]
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simulation_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/my_world.wbt']),
        ('share/' + package_name + '/protos', ['protos/SlugRobot.proto', 'protos/SolarCell.proto']),
        ('share/' + package_name + '/resource', ['resource/slugbot.urdf']),
        ('share/' + package_name + '/protos/textures', [
            'protos/textures/gold_leaf_base_color.jpg',
            'protos/textures/gold_leaf_roughness.jpg',
            'protos/textures/gold_leaf_normal.jpg',
            'protos/textures/gold_leaf_occlusion.jpg',
            'protos/textures/sojourner_metal.jpg',
        ]),
        ('share/' + package_name + '/protos/textures/solar_cell', [
            'protos/textures/solar_cell/solar_cell_base_color.jpg',
            'protos/textures/solar_cell/solar_cell_metalness.png',
            'protos/textures/solar_cell/solar_cell_normal.png',
            'protos/textures/solar_cell/solar_cell_occlusion.png',
            'protos/textures/solar_cell/solar_cell_roughness.jpg',
        ]),
    ],
)
