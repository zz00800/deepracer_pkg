from setuptools import setup, find_packages

package_name = 'deepracer_path_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sanghoon',
    maintainer_email='zz00800@yonsei.ac.kr',
    description= 'path_following using pure pursuit',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'deepracer_path_following = deepracer_path_following.path_following.path_following:main'
        ],
    },
)
