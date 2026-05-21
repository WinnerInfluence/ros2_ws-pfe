from setuptools import find_packages, setup

package_name = "safe_drl_nav"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML>=5.4"],
    zip_safe=True,
    maintainer="Researcher",
    maintainer_email="researcher@todo.todo",
    description="Safe DRL Agent with Rollback",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safe_agent = safe_drl_nav.main_agent:main",
            "hot_swap_eval = safe_drl_nav.hot_swap_eval_node:main",
            "chaos_spawner = safe_drl_nav.dynamic_obstacle:main",
            "verify_cloud_ready = safe_drl_nav.verify_cloud_readiness:main",
        ],
    },
    package_data={
        package_name: [
            "sim_assets/worlds/*",
            "sim_assets/scripts/*.py",
            "sim_assets/models/*",
            "training_contract.yaml",
        ],
    },
)
