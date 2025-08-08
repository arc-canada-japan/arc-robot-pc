from glob import glob
import os
from setuptools import find_packages, setup
from itertools import chain

package_name = CURRENT_PACKAGE = os.path.basename(os.path.dirname(os.path.abspath(__file__)))

# Function to find all controller scripts
def find_controllers():
    controllers = []
    for filepath in glob(os.path.join(package_name, '*_controller.py')):
        filename = os.path.basename(filepath)
        modulename = filename[:-3]  # Strip .py extension
        if modulename != 'abstract_controller':
            controllers.append(f"{modulename} = {package_name}.{modulename}:main")
    return controllers

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files.
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Include all image files.
        (os.path.join('share', package_name, 'image'), list(chain.from_iterable(glob(os.path.join('image', ext)) for ext in ('*.png', '*.jpg', '*.jpeg'))))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yoshida Lab (Siméon Capy)',
    maintainer_email='simeon.capy@rs.tus.ac.jp',
    description='Main package of the ARC project, control the robot and call the other packages',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': find_controllers(),
    },
)
