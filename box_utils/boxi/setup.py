from setuptools import find_packages, setup

if __name__ == "__main__":
    setup(
        name="boxi",
        author="Jonas Frey",
        version="1.0.0",
        description="Easy development tools for the box",
        author_email="jonfrey@ethz.ch",
        packages=find_packages(),
        include_package_data=True,
        package_data={"boxi": ["*/*.so"]},
        classifiers=[
            "Development Status :: 0 - Beta",
            "License :: NaN",
            "Operating System :: OS Independent",
            "Programming Language :: Python :: 3",
            "Programming Language :: Python :: 3.6",
            "Programming Language :: Python :: 3.7",
            "Programming Language :: Python :: 3.8",
        ],
        license="TBD",
        ext_modules=[],
        cmdclass={},
        zip_safe=False,
        entry_points={
            "console_scripts": [
                "boxi = boxi.__main__:main",
            ]
        },
        install_requires=["argcomplete>=3.2.2", "setuptools>=45.2.0", "black==24.4.0", "ntplib"],
    )
