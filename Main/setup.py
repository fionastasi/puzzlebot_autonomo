from setuptools import find_packages, setup

package_name = 'pzb_final'

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
    maintainer='danieldrg',
    maintainer_email='dreggam@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller_node = pzb_final.main_controller_node:main',
            'odometry_node = pzb_final.odometry_node:main',
            'odometry_controller_node = pzb_final.odometry_controller_node:main',
            'line_follower_node = pzb_final.line_follower_node:main',
            'sign_detection_node = pzb_final.sign_detection_node:main',
            'color_detection_node = pzb_final.color_detection_node:main',
            'video_capture = pzb_final.video_capture:main',
        ],
    },
)
