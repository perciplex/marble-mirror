from setuptools import setup

setup(
    name="marble_mirror",
    version="0.1",
    # py_modules=["marble_mirror"],
    packages=["src"],
    install_requires=[
        "Click",
    ],
    entry_points="""
        [console_scripts]
        mm=src.marble_mirror:cli
    """,
)
