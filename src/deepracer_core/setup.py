import glob, os
from setuptools import setup, find_packages

package_name = 'deepracer_core'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanghoon',
    maintainer_email='zz00800@yonsei.ac.kr',
    description='deepracer basic core node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepracer_core = deepracer_core.core.core:main',
        ],
    },
)
