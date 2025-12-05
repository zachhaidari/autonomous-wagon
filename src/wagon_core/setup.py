from setuptools import setup, find_packages

package_name = 'wagon_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zach',
    maintainer_email='zach@example.com',
    description='Core wagon coordinator and system management',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wagon_coordinator=wagon_core.wagon_coordinator:main',
            'calibration_tool=wagon_core.calibration_tool:main',
        ],
    },
)
