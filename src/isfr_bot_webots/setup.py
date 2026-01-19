from setuptools import find_packages, setup
import os

package_name = 'isfr_bot_webots'

def data_folder(pkg_name, folder):
    files = []
    for dp, dn, filenames in os.walk(folder):
        for f in filenames:
            path = os.path.join(dp, f)
            if os.path.isfile(path):  # skip non-files
                files.append(path)
    return ('share/' + f"{pkg_name}/{folder}", files)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        data_folder(package_name, "launch"),
        data_folder(package_name, "worlds"),
        data_folder(package_name, "controllers"),
        data_folder(package_name, "config"),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebbew',
    maintainer_email='ebbe.wertz8@gmail.com',
    description='Webots simulation and controllers launcher',
    license='Apache License 2.0',
    extras_require={},
    entry_points={
        'console_scripts': [
            'launch.frontend.launch_extension = launch_ros.launch.frontend.launch_extension',
            'ground_truth_odom = isfr_bot_webots.ground_truth_odom_publisher:main',
            'twist_stamper = isfr_bot_webots.twist_stamper:main'
        ],
    },
)
