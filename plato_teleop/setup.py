from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'plato_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dk',
    maintainer_email='dongho@utexas.edu',
    description='PLATO teleoperation package with spacemouse and other control interfaces',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'spacemouse_twist = plato_teleop.spacemouse_twist:main',
            'spacemouse_state_machine = plato_teleop.spacemouse_state_machine:main',
        ],
    },
)
