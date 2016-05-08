 #!/usr/bin/env python
 # -*- coding: utf-8 -*-

import os
from distutils.core import setup
import glob

setup(name='mutualModelling',
      version='0.0',
      license='ISC',
      description='library of functions/algorithms to build, update and compaire models of different agents built by a robot',
      author='Alexis Jacq',
      author_email='alexis.jacq@gmail.ch',
      package_dir = {'': 'src'},
      packages=['mutualModelling'],
      data_files=[('share/doc/mutualModelling', ['AUTHORS', 'LICENSE', 'README.md'])]
      )
