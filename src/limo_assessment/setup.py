from setuptools import find_packages, setup

package_name = 'limo_assessment'

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
    maintainer='adeola',
    maintainer_email='adeola@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "pothole_counter = limo_assessment.pothole_counter:main",
            "autonomous_navigation = limo_assessment.autonomous_navigation:main"
        ],
    },
)
#packagename.filename:functionname