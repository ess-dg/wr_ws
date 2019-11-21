#!/usr/bin/env python

import sys

from welder_node_gui.welder_gui import MyPlugin
from rqt_gui.main import Main

plugin = 'welder_node_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))