from setuptools import setup

package_name = "carbase_package"
modules = f"{package_name}/modules"
carbase_controller_module = f"{modules}/carbase_controller"

setup(
    name = package_name,
    version = "0.0.0",
    packages = [package_name, modules, carbase_controller_module],
    data_files = [
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires = ["setuptools"],
    zip_safe = True,
    maintainer = "ee3070group1",
    maintainer_email = "ee3070group1@todo.todo",
    description = "TODO: Package description",
    license = "TODO: License declaration",
    tests_require = ["pytest"],
    entry_points = {
        "console_scripts": [
            "carbase_node = carbase_package.carbase_node:main"
        ],
    },
)
