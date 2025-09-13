from setuptools import find_packages, setup

package_name = 'turtle_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/launch_me.py']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoyo',
    maintainer_email='youssifalsayad5656@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawn_base_turtle = turtle_project.main_pub_node:main',
            'chase_turtle_mov = turtle_project.chase_turtle_mov:main',
            'hunter_turtle = turtle_project.hunter_mov:main',
        ],
    },
)
