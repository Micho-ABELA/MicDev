from qt_gui.plugin import Plugin
from .master_gui_interface import MyWidget


class AirplanePlugin(Plugin):

    def __init__(self, context):
        super(AirplanePlugin, self).__init__(context)
        # Create QWidget
        self._widget = MyWidget(context)  # Give context to Widget file
        self._widget.Exit_Button.clicked[bool].connect(self.shutdown_plugin)  # EXIT button function
        # Add widget to the user interface
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
