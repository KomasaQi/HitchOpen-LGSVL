from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'competition_timer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pymap3d'],
    zip_safe=True,
    maintainer='roar-tl',
    maintainer_email='ztl1998@berkeley.edu',
    description='Competition Timer',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'competition_timer = competition_timer.competition_timer:main',
        ],
    },
)
