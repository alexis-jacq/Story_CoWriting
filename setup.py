 #!/usr/bin/env python
 # -*- coding: utf-8 -*-

import os
from distutils.core import setup
import glob

setup(name='NaoStoryTelling',
      version='0.0',
      license='ISC',
      description='library of random story-telling-like gestures for Nao robot',
      author='Alexis Jacq',
      author_email='alexis.jacq@gmail.com',
      package_dir = {'': 'src'},
      packages=['NaoStoryTelling'],
      data_files=[('share/doc/NaoStoryTelling', ['AUTHORS', 'LICENSE', 'README.md'])]
      )
