#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

from setuptools import setup

package_name = 'o3de_kraken_orchestration'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alekkam',
    maintainer_email='aleksander.kaminski@robotec.ai',
    description='Orchestration code for AppleKraken robot',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kraken_orchestration_node = '
            'o3de_kraken_orchestration.global_kraken_orchestration:main'
        ],
    },
)
