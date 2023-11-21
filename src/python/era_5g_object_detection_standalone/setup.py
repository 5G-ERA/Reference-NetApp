#!/usr/bin/env python

from setuptools import setup

setup(
    name="era_5g_object_detection_standalone",
    version="0.2.0",
    description="Standalone variant of object detection NetApp",
    author="Michal Kapinus",
    author_email="ikapinus@fit.vutbr.cz",
    packages=["era_5g_object_detection_standalone"],
    entry_points={
        "console_scripts": [
            "era_5g_object_detection_standalone = era_5g_object_detection_standalone.interface:main",
        ],
    },
)
