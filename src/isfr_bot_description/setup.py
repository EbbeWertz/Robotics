from setuptools import find_packages, setup

package_name = 'isfr_bot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/protos', [
            'protos/openmanipulator_x.proto',
            'protos/turtlebot3_waffle.proto'
        ]),
        ('share/' + package_name + '/urdf', [
            'urdf/openmanipulator_x.urdf',
            'urdf/turtlebot3_waffle.urdf'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebbew',
    maintainer_email='ebbe.wertz8@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
