from setuptools import find_packages, setup
import os

package_name = 'isfr_bot_description'

# Helper to recursively gather all files in a folder
def get_package_files(folder):
    return [os.path.join(dp, f) for dp, dn, filenames in os.walk(folder) for f in filenames]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all proto files automatically
        ('share/' + package_name + '/protos', get_package_files('protos')),
        # Include all urdf files automatically
        ('share/' + package_name + '/urdf', get_package_files('urdf')),
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
