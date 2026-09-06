#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from pickup_behaviors_node import CheckObject, GetObject, LetObject, main

if __name__ == '__main__':
    main()