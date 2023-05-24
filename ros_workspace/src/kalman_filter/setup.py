from setuptools import find_packages, setup

package_name = 'kalman_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'filterpy',
        'rclpy',
        'geometry_msgs',
        'custom_interfaces',
        'csv'
    ],
    zip_safe=True,
    maintainer='johndoe',
    maintainer_email='57753615+nicnike@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalmanFilterNode = kalman_filter.kalmanFilterNode:main',
        ],
    },
)
