from setuptools import find_packages, setup

package_name = 'robotics_soln'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['FilterSensorVec3D.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mepix',
    maintainer_email='13284555+mepix@users.noreply.github.com',
    description='3-DOF Sensor Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor = robotics_soln.sensor:main',
            'sensor_service = robotics_soln.sensor_service:main',
            'example = robotics_soln.example:main',
        ],
    },
)
