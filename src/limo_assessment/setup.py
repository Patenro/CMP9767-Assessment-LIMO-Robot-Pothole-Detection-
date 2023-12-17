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
            "test_node = limo_assessment.my_first_node:main", 
            "pothole_detector = limo_assessment.pothole_detector:main",
            "draw_circle = limo_assessment.draw_circle:main",
            "navigation = limo_assessment.navigation:main",
            "HSV_pothole_detector = limo_assessment.pothole_detectorHSV:main",
        ],
    },
)
#packagename.filename:functionname