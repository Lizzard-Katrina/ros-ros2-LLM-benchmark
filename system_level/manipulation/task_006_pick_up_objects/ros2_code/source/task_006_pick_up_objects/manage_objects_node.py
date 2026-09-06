#!/usr/bin/env python3
# Thin wrapper that re-exports from the top-level module
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from manage_objects_node import ManageObject, main

if __name__ == '__main__':
    main()