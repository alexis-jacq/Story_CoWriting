 #!/usr/bin/env python
 # -*- coding: utf-8 -*-

import os
from distutils.core import setup
import glob

setup(name='mutual_modelling',
      version='0.0',
      license='ISC',
      description='library of functions/algorithms to build, update and compaire models of different agents built by a robot',
      author='Alexis Jacq',
      author_email='alexis.jacq@gmail.ch',
      package_dir = {'': 'src'},
      packages=['mutual_modelling'],
      data_files=[('share/doc/mutual_modelling', ['AUTHORS', 'LICENSE', 'README.md'])]
      )
