from setuptools import setup

setup(
    name="marble_mirror",
    version='0.1',
    py_modules=['marble_mirror'],
    install_requires=[
        'Click',
    ],
    entry_points='''
        [console_scripts]
        mm=marble_mirror:cli
    ''',
)