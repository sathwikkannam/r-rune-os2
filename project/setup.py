from setuptools import find_packages, setup

package_name = 'project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv/', [
            'GetBestNodeForExploration.srv',
            'PathPlanner.srv'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sathwik',
    maintainer_email='',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_srv = project.navigation:main',
            'explore_srv = project.exploration:main'
        ],
    },
)
