from setuptools import find_packages, setup
import os
import glob

package_name = 'master_application'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob.glob('config/*')),  # Install configs
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='logilab',
    maintainer_email='logilab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_agv = master_application.test_agv_client:main',
            'test_robot = master_application.test_arm_client:main',
            'test_storage = master_application.test_storage_position_handling:main',
            'test_pipeline = master_application.test_complete_pipeline:main',  
        ],
    },
)
