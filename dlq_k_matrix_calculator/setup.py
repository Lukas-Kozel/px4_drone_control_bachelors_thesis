from setuptools import find_packages, setup

package_name = 'dlq_k_matrix_calculator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','numpy'],
    zip_safe=True,
    maintainer='luky',
    maintainer_email='l.kozel@outlook.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dlq_k_matrix_calculator = dlq_k_matrix_calculator.dlq_k_matrix_calculator:main'
        ],
    },
)
