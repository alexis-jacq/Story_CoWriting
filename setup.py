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
setup(name='mtTkinter',
      version='0.0',
      license='ISC',
      description='''This module modifies the original Tkinter module in memory, making all
functionality thread-safe. It does this by wrapping the Tk class' tk
instance with an object that diverts calls through an event queue when
the call is issued from a thread other than the thread in which the Tk
instance was created. The events are processed in the creation thread
via an 'after' event.''',
      author='Allen B. Taylor',
      author_email='a.b.taylor@gmail.com',
      package_dir = {'': 'src'},
      packages=['mtTkinter'],
      data_files=[('share/doc/mtTkinter', ['AUTHORS', 'LICENSE', 'README.md'])]
      )
