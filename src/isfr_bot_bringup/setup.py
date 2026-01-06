from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'isfr_bot_bringup'

# Helper function to include all files in a folder
def get_package_files(folder):
    return [os.path.join(dp, f) for dp, dn, filenames in os.walk(folder) for f in filenames]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        ('share/' + package_name + '/launch', get_package_files('launch')),
        # Include all config files
        ('share/' + package_name + '/config', get_package_files('config')),
        # Include all worlds
        ('share/' + package_name + '/worlds', get_package_files('worlds')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebbew',
    maintainer_email='ebbe.wertz8@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
