from setuptools import find_packages, setup

package_name = 'turtle_control_Mariana'

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
    maintainer='mariana',
    maintainer_email='marianargdy@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_control = turtle_control_Mariana.turtle_control:main',
            'goal_manager = turtle_control_Mariana.goal_manager:main'
        ],
    },
)
