from setuptools import find_packages, setup

package_name = 'view_radarscan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch file
        ('share/' + package_name + '/launch', ['launch/view_radarscan.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoshiri',
    maintainer_email='yoshiyoshidetteiu@gmail.com',
    description='Visualize radarscan with MarkerAarray',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'show_radar_scan = view_radarscan.show_radar_scan:main',
        ],
    },
)
