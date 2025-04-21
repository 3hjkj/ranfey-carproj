# -*- coding: utf-8 -*-
"""
Created on Sat Jun 27 20:53:57 2020

@author: Administrator
"""

import numpy as np
from distutils.core import setup
from Cython.Build import cythonize

setup(name='lidarcluster',
      ext_modules=cythonize("lidarcluster.pyx"),
      include_dirs=[np.get_include()])
