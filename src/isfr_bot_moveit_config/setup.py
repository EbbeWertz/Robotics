from setuptools import find_packages, setup

package_name = 'isfr_bot_moveit_config'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', [
        'launch/moveit_launch.py',
        'launch/open_manipulator_x_moveit.launch.py'
    ]),
    ('share/' + package_name + '/config', [
        'config/kinematics.yaml',
        'config/chomp_planning.yaml',
        'config/ompl_planning.yaml',
        'config/pilz_cartesian_limits.yaml',
        'config/pilz_industrial_motion_planner_planning.yaml',
        'config/moveit.rviz'
    ])],
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
