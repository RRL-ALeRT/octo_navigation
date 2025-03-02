from setuptools import find_packages, setup

package_name = 'pure_pursuit_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='max',
    maintainer_email='m.kirsch@fh-aachen.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obsstacle_avoidance_path_to_vel = pure_pursuit_controller.obstacle_avoidance_path_to_vel:main',
        ],
    },
)
