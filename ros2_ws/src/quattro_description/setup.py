from setuptools import setup
from glob import glob
import os

package_name = 'quattro_description'

setup(
	name=package_name,
	version='0.0.1',
	packages=[package_name],
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
		(os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
		(os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'stl'), glob('stl/*')),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='Taejin Jo',
	maintainer_email='jtjin0916@gmail.com',
	description='QUATTRO quadruped robot description package',
	license='Apache-2.0',
	tests_require=['pytest'],
	entry_points={'console_scripts': []},
)
