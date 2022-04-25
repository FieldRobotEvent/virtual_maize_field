#!/usr/bin/env python3
from __future__ import annotations

from datetime import datetime, timedelta
from pathlib import Path
from socket import timeout
from urllib import request
from xml.etree import ElementTree

import rospkg
import rospy
from packaging import version

GITHUB_PACKAGE_URL = "https://raw.githubusercontent.com/FieldRobotEvent/virtual_maize_field/main/package.xml"


def version_from_xml(xml_data: str) -> version.Version:
    root = ElementTree.fromstring(xml_data)
    return version.parse(root.findtext("version"))


def remote_version_from_cache(cache_file: Path) -> version.Version | None:
    if not cache_file.is_file():
        return None

    cache_content = cache_file.read_text(encoding="utf-8").splitlines()
    if len(cache_content) != 2:
        return None

    last_check = datetime.strptime(cache_content[0], "%d-%b-%Y (%H:%M:%S.%f)")
    if last_check + timedelta(days=1) >= datetime.now():
        return None

    return version.parse(cache_content[1])


def remote_version_to_cache(cache_file: Path, version: version.Version) -> None:
    timestamp = datetime.now().strftime("%d-%b-%Y (%H:%M:%S.%f)")
    cache_file.write_text(f"{timestamp}\n{version}", encoding="utf-8")


def main() -> None:
    rospy.init_node("virtual_maize_field_update_checker")

    cache_file = Path("/tmp/virtual_maize_field_update_check.tmp")
    remote_version = remote_version_from_cache(cache_file)

    if remote_version is None:
        try:
            # Get the remote version
            remote_version_xml_data = request.urlopen(
                GITHUB_PACKAGE_URL, timeout=2
            ).read()
            remote_version = version_from_xml(remote_version_xml_data)
            remote_version_to_cache(cache_file, remote_version)

        except timeout:
            rospy.logdebug(
                "Could not get remote version of the 'virtual_maize_field' package. Probably because there is no internet available."
            )

        except Exception:
            rospy.loginfo("Could not check version of 'virtual_maize_field' package.")

    # Get the local version
    local_package_xml = rospkg.RosPack().get_path("virtual_maize_field")
    local_package_xml_data = (Path(local_package_xml) / "package.xml").read_text(
        encoding="utf-8"
    )
    local_version = version_from_xml(local_package_xml_data)

    # Print error if we have an old local version
    if local_version < remote_version:
        rospy.logwarn(
            f"Your 'virtual_maize_field' package is outdated ({local_version} < {remote_version})! Run 'git submodule update --remote --merge' to update the package and generate new worlds."
        )


if __name__ == "__main__":
    main()
