import warnings
from setuptools import find_packages, setup
from setuptools.command.develop import develop as _develop
import os
from glob import glob

package_name = 'plato_teleop'


# Silence setuptools develop deprecation noise emitted by colcon's invocation.
try:
    from setuptools import SetuptoolsDeprecationWarning
except ImportError:  # pragma: no cover - fallback for older setuptools
    class SetuptoolsDeprecationWarning(Warning):
        pass
warnings.filterwarnings("ignore", category=SetuptoolsDeprecationWarning)


class DevelopCommand(_develop):
    """Add a no-op --editable flag so colcon's invocation succeeds."""

    user_options = _develop.user_options + [
        ('editable', None, 'Editable install (ignored)'),
        ('build-directory=', None, 'Build directory (ignored)'),
        ('script-dir=', None, 'Script install directory (ignored)'),
    ]
    boolean_options = _develop.boolean_options + ['editable']

    def initialize_options(self):
        super().initialize_options()
        self.editable = True
        self.build_directory = None
        self.script_dir = None


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
            'impedance_gain_keyboard = plato_teleop.impedance_gain_keyboard:main',
            'grasp_task_keyboard = plato_teleop.grasp_task_keyboard:main',
            'retargeting_converter = plato_teleop.retargeting_converter:main',
            'ftip_to_parallel_grasp = plato_teleop.ftip_to_parallel_grasp:main',
        ],
    },
    cmdclass={'develop': DevelopCommand},
)
