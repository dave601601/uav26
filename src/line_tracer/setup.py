from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'line_tracer'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dongha Gwak',
    maintainer_email='noreply@todo.todo',
    description='Indoor line-tracing + dead-reckoning controller using a downward-facing RealSense D435.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_tracer_node = line_tracer.line_tracer_node:main',
        ],
    },
)
