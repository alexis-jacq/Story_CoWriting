 #!/usr/bin/env python
 # -*- coding: utf-8 -*-

import os
from distutils.core import setup
import glob

setup(name='naoStoryTelling',
      version='0.0',
      license='ISC',
      description='library of random story-telling-like gestures for Nao robot',
      author='Alexis Jacq',
      author_email='alexis.jacq@gmail.com',
      package_dir = {'': 'src'},
      packages=['naoStoryTelling'],
      data_files=[('share/doc/naoStoryTelling', ['AUTHORS', 'LICENSE', 'README.md'])]
      )
setup(name='nextChoice',
      version='0.0',
      license='ISC',
      description='choosing next story object with simple ToM2',
      author='Alexis Jacq',
      author_email='alexis.jacq@gmail.com',
      package_dir = {'': 'src'},
      packages=['nextChoice'],
      data_files=[('share/doc/nextChoice', ['AUTHORS', 'LICENSE', 'README.md'])]
      )
