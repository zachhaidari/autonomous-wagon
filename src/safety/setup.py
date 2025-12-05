from setuptools import setup, find_packages

package_name = 'safety'

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
    description='Safety manager and obstacle detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_manager=safety.safety_manager:main',
            'obstacle_detector=safety.obstacle_detector:main',
        ],
    },
)
