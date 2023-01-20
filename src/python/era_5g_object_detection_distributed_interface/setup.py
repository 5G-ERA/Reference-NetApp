#!/usr/bin/env python

from distutils.core import setup

setup(name='era_5g_object_detection_distributed',
      version='0.0.1',
      description='An interface for distributed variant of object detection 5G-ERA NetApp',
      author='Michal Kapinus',
      author_email='ikapinus@fit.vutbr.cz',
      packages=['era_5g_object_detection_distributed_interface'],
      entry_points = {
              'console_scripts': [
                  'era_5g_object_detection_distributed_interface = era_5g_object_detection_distributed_interface.interface:main',                  
              ],              
          },
     )
