from setuptools import find_packages, setup
import os
package_name = 'detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='src'),
    package_dir={'': 'src'},  
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/camera.launch.py']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cromanma',
    maintainer_email='cromanma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = detection.scripts.camera_node:main',
            'fuse_image = detection.scripts.fuse_image:main',
        ],
    },
)
