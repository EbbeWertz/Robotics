from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'isfr_bot_security'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    
        # --- ADD THIS LINE BELOW ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='mathias.houwen@student.uhasselt.be',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_sentry = isfr_bot_security.depth_sentry:main',
        ],
    },
)
