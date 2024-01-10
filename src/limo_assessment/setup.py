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
            "pothole_counter = limo_assessment.adeola_pothole_counter:main",
            "autonomous_navigation = limo_assessment.adeola_autonomous_navigation:main",
            "autonomous_navigation1 = limo_assessment.adeola_autonomous_navigation2:main",
            "adeola_marker = limo_assessment.adeola_marker:main",
            "adeola_posedetector = limo_assessment.adeola_posedetector1:main",
            "adeola_posedetector2 = limo_assessment.adeola_posedetector2:main",
            "adeola_severity_report = limo_assessment.adeola_severity_report:main",
            
        ],
    },
)
#packagename.filename:functionname