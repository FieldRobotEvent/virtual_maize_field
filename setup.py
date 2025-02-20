from __future__ import annotations

from os import path, walk
from pathlib import Path
from xml.etree import ElementTree
from glob import glob
from setuptools import find_packages, setup

package_data = ElementTree.parse(Path(__file__).parent / "package.xml")
package_name = package_data.find("./name").text


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    ('share/' + package_name+'/models/', glob('models/*')),
    ('share/' + package_name+'/models/meshes', glob('models/meshes/*')),
]

# Add all folders recursively
# https://answers.ros.org/question/397319/how-to-copy-folders-with-subfolders-to-package-installation-path/
for folder_name in (
    "config",
    "launch",
    "Media",
    "models",
):
    _path_dict = {}

    for path_, directories, filenames in walk(folder_name):
        for filename in filenames:
            file_path = path.join(path_, filename)
            install_path = path.join("share", package_name, path_)

            if install_path in _path_dict.keys():
                _path_dict[install_path].append(file_path)

            else:
                _path_dict[install_path] = [file_path]

    for key in _path_dict.keys():
        data_files.append((key, _path_dict[key]))


setup(
    name=package_name,
    version=package_data.find("./version").text,
    packages=find_packages(),
    data_files=data_files,
    package_data={"": ["*.template"]},
    include_package_data=True,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer=package_data.find("./maintainer").text,
    maintainer_email=package_data.find("./maintainer").attrib["email"],
    description=package_data.find("./description").text,
    license=package_data.find("./license").text,
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "generate_world = virtual_maize_field.generate_world:main",
        ],
    },
)
