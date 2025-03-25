from setuptools import setup

package_name = 'print_tf'

setup(
    name = package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='h16',
    maintainer_email='gzc2035@163.com',
    description='TF Listener Node in Python',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'print_tf = print_tf.print_tf:main',
        ],
    },
)