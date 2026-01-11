from setuptools import setup
import os

if __name__ == '__main__':
    # compute absolute file URLs to subpackages
    root = os.path.dirname(__file__)
    droid_path = os.path.abspath(os.path.join(root, 'droid'))
    oculus_path = os.path.abspath(os.path.join(root, 'oculus_reader'))

    # PEP 508 direct references (pip will install from these paths)
    install_requires = [
        'numpy',
        'hydra-core',
        'omegaconf',
        f"droid @ file://{droid_path}",
        f"oculus_reader @ file://{oculus_path}",
    ]

    setup(
        setup_requires=['setuptools>=40.8.0'],
        install_requires=install_requires,
    )
