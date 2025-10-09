from setuptools import setup, find_packages

setup(
    name='dp-opt',
    package_dir={"":"src"},
    py_modules=[              
        'run_bay_opt',
        'csv2rksml',
        'config_gui',
        'gen_json',
        'gen_products',
    ],
    entry_points={
        'console_scripts': [
            'dp-opt=run_bay_opt:main', 
            'dp-csv2rksml=csv2rksml:main',
            'dp-gui=config_gui:main' 
        ],
    },
    install_requires=[
        'requests',
        'optuna',
        'optuna-dashboard',
        'matplotlib',
        'pandas',
        'defusedxml'
        #'tkinter'
    ],
)

