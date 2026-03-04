from setuptools import find_packages, setup

package_name = 'bag_to_video'

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
    maintainer='caillotantoine',
    maintainer_email='antoine.caillot@aist.go.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_to_video = bag_to_video.bag_to_video:main',
        ],
    },
)
