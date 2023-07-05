import os
import rospy
import rospkg
import threading
from std_srvs.srv import Trigger
import any_msgs.srv

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtCore import Slot


class RecordingPlugin(Plugin):
    def __init__(self, context):
        super(RecordingPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName("RecordingPlugin")
        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path("rqt_recording_plugin"), "resource", "recording.ui")
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Robot selection
        self.topic_prefix = ""
        self.selected_input = 0
        self.anymal_prefixes = rospy.get_param(
            "~prefixes", ["/anymal_1", "/anymal_2", "/anymal_3", "/anymal_4", "/anymal_5", "/anymal_6"]
        )
        self.anymal_selection_server = rospy.Service(
            "/recording_plugin/select", any_msgs.srv.SetUInt32, self.anymal_selection_callback
        )

        self.start_recording_topic = "/rosbag_record_robot_coordinator/record_bag"
        self.stop_recording_topic = "/rosbag_record_robot_coordinator/stop_bag"

        self.start_recording_proxy = rospy.ServiceProxy(self.start_recording_topic, Trigger)
        self.stop_recording_proxy = rospy.ServiceProxy(self.stop_recording_topic, Trigger)

        self._widget.push_button_start_recording.clicked.connect(self.start_recording_button_slot)
        self._widget.push_button_stop_recording.clicked.connect(self.stop_recording_button_slot)

    def shutdown_plugin(self):
        self.start_recording_proxy.close()
        self.stop_recording_proxy.close()
        self.anymal_selection_server.shutdown()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    @Slot()  # Button callback function
    def start_recording_button_slot(self):
        self.start_recording = threading.Thread(target=self.start_recording_thread)
        self.start_recording.start()

    def start_recording_thread(self):
        response = self.start_recording_proxy()
        if response.success:
            rospy.loginfo("[Recording Plugin] Successfully triggered rosbag recording.")
        else:
            rospy.logwarn("[Recording Plugin] Rosbag could not be started")
            rospy.logwarn(response.message)

    @Slot()  # Button callback function
    def stop_recording_button_slot(self):
        self.stop_recording = threading.Thread(target=self.stop_recording_thread)
        self.stop_recording.start()

    def stop_recording_thread(self):
        response = self.stop_recording_proxy()
        if response.success:
            rospy.loginfo("[Recording Plugin] Successfully triggered stop rosbag recording.")
        else:
            rospy.logwarn("[Recording Plugin] Rosbag could not be stopped")
            rospy.logwarn(response.message)

    def anymal_selection_callback(self, request):
        response = any_msgs.srv.SetUInt32Response()
        requested_input_index = request.data
        if requested_input_index >= len(self.anymal_prefixes):
            rospy.logerr(
                "[Recording Plugin] Requested input index %d is out of prefixes size %d."
                % (requested_input_index, len(self.anymal_prefixes))
            )
            response.success = False
        else:
            if self.selected_input != requested_input_index:
                self.selected_input = requested_input_index
                self.topic_prefix = self.anymal_prefixes[request.data]
                response.success = True
                prefixed_start_recording_topic = self.topic_prefix + self.start_recording_topic
                prefixed_stop_recording_topic = self.topic_prefix + self.stop_recording_topic

                self.start_recording_proxy.close()
                self.start_recording_proxy = rospy.ServiceProxy(prefixed_start_recording_topic, Trigger)

                self.stop_recording_proxy.close()
                self.stop_recording_proxy = rospy.ServiceProxy(prefixed_stop_recording_topic, Trigger)

            else:
                rospy.logwarn(
                    "[Recording Plugin] Requested input index %d is already selected." % requested_input_index
                )
                response.success = False
        return response
