from setuptools import setup, find_packages

setup(
    name='oneaxiscontrol',
    version='0.1',
    packages=find_packages(),
    package_dir={'': '.'},  # Important for non-standard structure
    install_requires=[
        'numpy',  # Add any dependencies here
    ],
    python_requires='>=3.6',
)