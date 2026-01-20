from setuptools import setup
import os               # <--- TOEVOEGEN
from glob import glob   # <--- TOEVOEGEN

package_name = 'isfr_bot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # --- VOEG DEZE REGEL TOE ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robin',
    maintainer_email='robin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = isfr_bot_vision.3d_yolo_detector:main',
            'marker_publisher = isfr_bot_vision.marker_publisher:main',
            'object_analyser = isfr_bot_vision.object_analyser:main'
        ],
    },
)