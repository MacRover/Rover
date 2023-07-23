#!/usr/bin/env python

from qt_gui.plugin import Plugin
from .plugin_widget import ScienceWidget

class SciencePlugin(Plugin):

    def __init__(self, context):
      super(SciencePlugin, self).__init__(context)
      # Give QObjects reasonable names
      self.setObjectName('SciencePlugin')

      self._widget = ScienceWidget(context)
      if context.serial_number() > 1:
                  self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
      context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()