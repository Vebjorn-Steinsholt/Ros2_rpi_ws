from setuptools import find_packages, setup

package_name = 'rpi_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MCL1',
    maintainer_email='MCL1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "temperature_sensor = rpi_tests.temperature_sensor:main",
            "create_folder_server = rpi_tests.create_folder_server:main",
        ],
    },
)
