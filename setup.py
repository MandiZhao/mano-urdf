from setuptools import find_packages, setup
import warnings

DEPENDENCY_PACKAGE_NAMES = [
    "numpy",
    "trimesh",
    "chumpy",
    "transforms3d",
    "urdfpy",
    "pyrender@git+https://github.com/mmatl/pyrender.git",
    ]

def check_dependencies():
    missing_dependencies = []
    for package_name in DEPENDENCY_PACKAGE_NAMES:
        try:
            __import__(package_name)
        except ImportError:
            missing_dependencies.append(package_name)

    if missing_dependencies:
        warnings.warn("Missing dependencies: {}.")


with open("README.md", "r") as fh:
    long_description = fh.read()

check_dependencies()

setup(
    name="mano_urdf",
    version="0.0.1",
    author="Wei Yang",
    author_email="weiy@nvidia.com",
    packages=find_packages(exclude=("examples")),
    python_requires=">=3.6.0",
    description="MANO URDF",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://gitlab-master.nvidia.com/weiy/mano_urdf",
    install_requires=DEPENDENCY_PACKAGE_NAMES,
    classifiers=[
        "Programming Language :: Python :: 3",
        "Operating System :: OS Independent",
    ],
)