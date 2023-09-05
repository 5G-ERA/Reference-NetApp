from setuptools import setup

package_name = 'era_5g_object_detection'

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
    maintainer='Petr Kleparnik',
    maintainer_email='p.kleparnik@cognitechna.cz',
    description='5G-ERA ROS2 object detection',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'era_5g_object_detection_node = era_5g_object_detection.era_5g_object_detection_node:main',

        ],
    },
)
