from setuptools import find_packages, setup

package_name = 'its_subcribernode'

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
    maintainer='sanjevanr_',
    maintainer_email='sanjevanr_@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'talker = its_subcribernode.publisher_member_function:main', 
                'listener = its_subcribernode.subscriber_member_function:main',
                'modified_publisher = its_subcribernode.modifiedpublisherfunction:main',
                'modified_subscriber = its_subcribernode.modifiedsubscriberfunction:main',
                'logical_loop = its_subcribernode.logicalloop:main_loop',
                'out = its_subcribernode.publishers.out:main',
                ],},
)
