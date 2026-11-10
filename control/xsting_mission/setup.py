from glob import glob

from setuptools import setup

package_name = "xsting_mission"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.pgm")),
        ("share/" + package_name + "/web", glob("web/*.html")),
        ("share/" + package_name + "/prompts", glob("prompts/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="minsikim-42",
    maintainer_email="spalstlr321@naver.com",
    description="주사위 기억 로봇 임무 판단기와 로봇 명령 서버",
    license="MIT",
    entry_points={
        "console_scripts": [
            "llm_commander = xsting_mission.llm_commander:main",
            "robot_command_server = xsting_mission.robot_command_server:main",
        ],
    },
)
