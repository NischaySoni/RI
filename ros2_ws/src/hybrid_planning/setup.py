import os
from glob import glob
from setuptools import setup

package_name = 'hybrid_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include PDDL files
        (os.path.join('share', package_name, 'pddl'),
            glob(os.path.join('pddl', '*.pddl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Hybrid TAMP with ROSPlan2 and MoveIt',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'monitor_motion = hybrid_planning.monitor_motion:main',
            'replanner = hybrid_planning.replanner:main',
            'sequential_controller = hybrid_planning.sequential_controller:main',
        ],
    },
)