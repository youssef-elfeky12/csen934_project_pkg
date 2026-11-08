from setuptools import find_packages, setup

package_name = 'csen934_project_pkg'

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
    maintainer='youssef_elfeky',
    maintainer_email='youssef_elfeky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'control = csen934_project_pkg.control:main',
            	'localization_add_noise = csen934_project_pkg.localization_add_noise:main',
            	'localization_ekf_node = csen934_project_pkg.localization_ekf_node:main',
            	'teleop = csen934_project_pkg.teleop:main',       	
        ],
    },
)
