from setuptools import setup
from glob import glob

package_name = 'o3de_kraken_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/launch/config", glob('launch/config/*.yaml')),
        ('share/' + package_name + "/launch/config", glob('launch/config/*.xml')),
        ('share/' + package_name + "/launch/config", glob('launch/config/*.rviz')),
        ('share/' + package_name + "/launch", glob('launch/*.launch.py')),
        ('share/' + package_name + "/launch", glob('launch/*.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Piotr Jaroszek',
    maintainer_email='piotr.jaroszek@robotec.ai',
    description='Navigation package for Roscon apple kraken demo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_to_ackermann = o3de_kraken_nav.twist_to_ackermann:main'
        ],
    },
)
