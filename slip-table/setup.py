from setuptools import setup, find_packages

setup(
    name='dp-slip-table',
    py_modules=['dp_slip_table'],
    package_dir={"": "src"},     
    entry_points={
        'console_scripts': [
            'dp-table=run_table:main',
            'dp-table-gp=gp_table:main' 
        ],
    },
    install_requires=[
        'requests',
        'scikit-learn',
        'defusedxml'
    ],
)
