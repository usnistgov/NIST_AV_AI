from setuptools import find_packages, setup

package_name = 'cv_app'
submodules = 'cv_app/cv_head'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mnh20',
    maintainer_email='mnh20@todo.todo',
    description='TODO',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = cv_app.sensor_publisher:main',
            'sensor_subscriber = cv_app.sensor_subscriber:main',
            'speed_publisher = cv_app.speed_publisher:main',
            'speed_subscriber = cv_app.speed_subscriber:main'
        ],
    },
)
