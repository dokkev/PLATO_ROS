from setuptools import find_packages, setup

package_name = 'spaceMouse_navigator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haiyunzhang',
    maintainer_email='haiyunzhang@utexas.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spaceMouse_motionTest = spaceMouse_navigator.spaceMouse_motionTest:main',
            'spaceMouse_publisher = spaceMouse_navigator.spaceMouse_publisher:main'
        ],
    },
)
