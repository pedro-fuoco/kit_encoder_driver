import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kit_encoder_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pedro-fuoco',
    maintainer_email='pedrofuoco6@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_node = kit_encoder_driver.quadratic_encoder_node:main',
            'odometry_node = kit_encoder_driver.odometry_node:main',
            'transformations_node = kit_encoder_driver.transformations_node:main'
        ],
    },
)
