from setuptools import setup

package_name = 'nissan_bridge_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nissan_bridge team',
    maintainer_email='devnull@example.com',
    description='Python test GUI package for Nissan bridge validation.',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'nissan_bridge_gui = nissan_bridge_gui.gui_node:main',
        ],
    },
)
