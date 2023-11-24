from setuptools import setup

package_name = 'coordinate_frame'

setup(
  name=package_name,
  version='0.0.1',
  packages=[package_name],
  data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='luky',
  maintainer_email='l.kozel@outlook.cz',
  description='A ROS2 package for Coordinate Frame Publishing',
  license='Apache 2.0',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'coordinate_frame_publisher = coordinate_frame.coordinate_frame_publisher:main'
    ],
  },
)
