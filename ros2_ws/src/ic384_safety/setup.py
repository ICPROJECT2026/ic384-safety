from glob import glob
from setuptools import setup

setup(
    name='ic384_safety',
    version='0.1.0',
    packages=['ic384_safety'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ic384_safety']),
        ('share/ic384_safety', ['package.xml']),
        ('share/ic384_safety/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    extras_require={'person_detection': ['ultralytics']},
    zip_safe=True,
    maintainer='IC384 team',
    maintainer_email='team@example.com',
    description='RGB-D person detection and latched base stop prototype',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'person_safety = ic384_safety.person_safety:main',
        'safety_supervisor = ic384_safety.safety:main',
    ]},
)
