from setuptools import setup

package_name = 'era_5g_object_detection_standalone'

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
    maintainer='ispanhel',
    maintainer_email='ispanhel@fit.vutbr.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = era_5g_object_detection_standalone.basic_image_publisher:main',
            'result_listener = era_5g_object_detection_standalone.basic_result_listener:main',
            'ml_service = era_5g_object_detection_standalone.ml_service:main'
        ],
    },
)
