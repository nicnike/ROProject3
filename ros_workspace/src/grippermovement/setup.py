from setuptools import find_packages, setup

package_name = 'grippermovement'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[        
        'setuptools',
        'rclpy',
        'ro45_portalrobot_interfaces'
        'time'],
    zip_safe=True,
    maintainer='Ai Avengers',
    maintainer_email='57753615+nicnike@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MoveGripper = controller.MoveGripper:main',
        ],
    },
)
