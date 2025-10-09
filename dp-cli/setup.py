from setuptools import setup, find_packages

setup(
    name='drive-primer',
    py_modules=['script'],
    package_dir={"": "src"},     
    entry_points={
        'console_scripts': [
            'drive-primer=drive_primer:main', 
        ],
    },
    install_requires=[
        'requests',
    ],
)
