from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'isfr_bot_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebbew',
    maintainer_email='ebbe.wertz8@gmail.com',
    description='Inverse Kinematics for arm',
    license='Apache License 2.0',
    extras_require={},
    entry_points={
        'console_scripts': [
            'arm_ik = isfr_bot_manipulation.inverse_kinematics:main'
        ],
    },
)
