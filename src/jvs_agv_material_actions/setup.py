from setuptools import find_packages, setup

package_name = 'jvs_agv_material_actions'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/material_action.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hello',
    maintainer_email='todo@todo.com',
    description='PyQt5 GUI for material loading/unloading action simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'material_action_gui = jvs_agv_material_actions.material_action_gui:main',
        ],
    },
)
