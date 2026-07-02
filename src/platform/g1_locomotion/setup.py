import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'g1_locomotion'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Ship the vendored AMO policy assets alongside the installed package
        # so a colcon install / deployed wheel can find them. The runtime also
        # falls back to the in-source assets/ dir (see amo_policy_runtime.py).
        (os.path.join('share', package_name, 'assets'),
            glob(os.path.join(package_name, 'assets', '*.pt'))),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Gabriel Voss',
    maintainer_email='gabriel.voss01@gmail.com',
    description='Self-contained Unitree G1 AMO legs-only locomotion driver.',
    license='Proprietary',
    tests_require=['pytest'],
    # Ensure the .pt assets are packaged when building a wheel/sdist.
    include_package_data=True,
    package_data={package_name: ['assets/*.pt']},
    entry_points={
        'console_scripts': [
            # Main driver. Modes: live control, --observe_only, --offline_selftest.
            'amo_driver = g1_locomotion.amo_driver:main',
        ],
    },
)
