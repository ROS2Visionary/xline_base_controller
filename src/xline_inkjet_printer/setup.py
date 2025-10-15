from setuptools import setup

package_name = 'xline_inkjet_printer'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='xline',
    maintainer_email='maintainer@example.com',
    description='Inkjet printer control node (Python, ROS 2).',
    license='Proprietary',
    tests_require=['pytest'],
    package_data={
        package_name: ['config/*.yaml'],
    },
    entry_points={
        'console_scripts': [
            'inkjet_printer_node = xline_inkjet_printer.async_inkjet_node:main',
        ],
    },
)
