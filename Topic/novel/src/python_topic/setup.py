from setuptools import find_packages, setup

package_name = 'python_topic'

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
    maintainer='jszr',
    maintainer_email='jszr@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'novel_pub_node = python_topic.novel_pub_node:main',
            'novel_sub_node = python_topic.novel_sub_node:main',
        ],
    },
)
