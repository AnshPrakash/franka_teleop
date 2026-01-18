from setuptools import setup
import os
import glob

package_name = 'franka_teleop'



if __name__ == '__main__':
    # compute absolute file URLs to subpackages
    root = os.path.dirname(__file__)
    droid_path = os.path.abspath(os.path.join(root, 'droid'))
    oculus_path = os.path.abspath(os.path.join(root, 'oculus_reader'))

    # PEP 508 direct references (pip will install from these paths)
    install_requires = [
        'setuptools',
        'numpy',
        'hydra-core',
        'omegaconf',
        f"droid @ file://{droid_path}",
        f"oculus_reader @ file://{oculus_path}",
    ]

    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/**')),
        ('share/' + package_name, glob.glob('config/**')),
        ('share/' + package_name, glob.glob('urdf/**')),
        ('share/' + package_name, glob.glob('worlds/**')),
    ]


    setup(
        name=package_name,
        version='0.0.1',
        packages=[package_name],
        data_files=data_files,
        install_requires=install_requires,
        zip_safe=True,
        author='Ansh Prakash',
        author_email='todo@email.org',
        keywords=['ROS2'],
        classifiers=[
            'Intended Audience :: Developers',
            'Programming Language :: Python',
            'Topic :: Software Development',
        ],
        description='Franka Emika Panda Teleoperation using Oculus VR Controller',
        license='Apache License, Version 2.0',
        entry_points={
            'console_scripts': [
                'oculus_transforms_vrpolicy = franka_teleop.oculus_transforms_vrpolicy:main',
                'teleop_quest = franka_teleop.teleop_quest:main',
            ],
        },
    )
