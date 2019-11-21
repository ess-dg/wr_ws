import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QLabel, QPushButton
from python_qt_binding.QtGui import QImage, QPixmap
from python_qt_binding.QtCore import Qt,Signal,Slot, QObject
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import  String
import numpy as np
import cv2 as cv

welder_states = ["Idle","Moving CNC to Next CV Point", "Awaiting CV result",
                 "Awating Start Mark","Move to Point","Awating CNC to place",
                 "Awating Laser Sync"]

class MyPlugin(Plugin):

    def __init__(self, context):
        self.cmd_pub               =  rospy.Publisher("/welder/cmd", String, queue_size = 1)
        self.welder_status_sub     = rospy.Subscriber('/welder/status', String, self.status_callback, queue_size = 10)
        self.welder_time_sub       = rospy.Subscriber('/welder/time', String, self.time_callback, queue_size = 10)
        self.welder_progress_sub   = rospy.Subscriber('/welder/progress', String, self.progress_callback, queue_size = 10)
        self.image_sub             = rospy.Subscriber("/laser_ctrl/img_feed/compressed",
                                                      CompressedImage, self.image_callback, queue_size = 10)
        self.laser_state_sub       = rospy.Subscriber('/laser_ctrl/cmd', String, self.laser_state_clbk, queue_size = 10)
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('welder_node_gui')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('welder_node_gui'), 'resource', 'gui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Welder Robot GUI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        self._widget.startButton.clicked.connect(self.start_cnc)
        self._widget.stopButton.clicked.connect(self.stop_cnc)
        self._widget.timeLabel.setText('None')
        #self._widget.progressBar.setValue(0)
        #self.progressSignal = Signal(str)
        #self._widget.connect(self._widget,self.progressSignal,self.progressSlot)

    def start_cnc(self):

        self.cmd_pub.publish('1')


    def progressSlot(self,data):

        blade_count = int(ros_data.data)
        progress    = int(float(blade_count)/256*100)
        grids       = int((blade_count/16))
        self._widget.progressBar.setValue(progress)
        self._widget.bladeNumber.display(blade_count)       
        self._widget.gridNumber.display(grids)   


    def stop_cnc(self):

        self.cmd_pub.publish('0')
        self._widget.timeLabel.setText('None')

    def image_callback(self,ros_data):
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image  = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        height, width, channel = image.shape
        bytesPerLine = 3 * width
        qImage = QImage(image.data, width, height, bytesPerLine, QImage.Format_RGB888).rgbSwapped()
        pix    = QPixmap(qImage)
        h_label = self._widget.videoDisplay.height()
        w_label = self._widget.videoDisplay.width()      
        self._widget.videoDisplay.setPixmap(pix.scaled(w_label,h_label,Qt.KeepAspectRatio))
    
    def time_callback(self,ros_data):
        time_text = ros_data.data + ' s'
        self._widget.timeLabel.setText(time_text) 

    def progress_callback(self,ros_data):

        self.emit(Signal("changeUI(PyQt_PyObject)"), ros_data.data)


    def laser_state_clbk(self,ros_data):

        if ros_data.data == 's':
            self._widget.laserStatusLabel.setText('OFF')
        elif ros_data.data == 'f':
            self._widget.laserStatusLabel.setText('ON')

    def status_callback(self,ros_data):

        if ros_data.data != '0':
            self._widget.startButton.setEnabled(False)
            self._widget.stopButton.setEnabled(True)
        else:
            self._widget.startButton.setEnabled(True)
            self._widget.stopButton.setEnabled(False)
        self._widget.statusLabel.setText(welder_states[int(ros_data.data)])


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog