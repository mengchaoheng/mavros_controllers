from python_qt_binding.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
import rospy
from rqt_gui_py.plugin import Plugin
from rqt_reconfigure.param_widget import ParamWidget
from std_srvs.srv import SetBool


class TrajectoryReconfigurePlugin(Plugin):
    """Dynamic reconfigure with explicit trajectory start and stop actions."""

    def __init__(self, context):
        super(TrajectoryReconfigurePlugin, self).__init__(context)
        self.setObjectName('Trajectory Parameters')

        self._widget = QWidget()
        self._widget.setObjectName('TrajectoryParameters')
        self._widget.setWindowTitle('Trajectory Parameters')
        layout = QVBoxLayout(self._widget)

        controls = QHBoxLayout()
        self._start_button = QPushButton('Start Trajectory')
        self._start_button.setStyleSheet(
            'QPushButton { background-color: #2e7d32; color: white; font-weight: bold; padding: 6px 18px; }'
        )
        self._stop_button = QPushButton('Stop Trajectory')
        self._stop_button.setStyleSheet(
            'QPushButton { background-color: #c62828; color: white; font-weight: bold; padding: 6px 18px; }'
        )
        self._status = QLabel('Standby: holding current position')
        controls.addWidget(self._start_button)
        controls.addWidget(self._stop_button)
        controls.addWidget(self._status, 1)
        layout.addLayout(controls)

        self._param_widget = ParamWidget(context, node='/trajectory_publisher')
        layout.addWidget(self._param_widget, 1)

        self._start_button.clicked.connect(lambda: self._set_tracking(True))
        self._stop_button.clicked.connect(lambda: self._set_tracking(False))

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                '{} ({})'.format(self._widget.windowTitle(), context.serial_number())
            )
        context.add_widget(self._widget)

    def _set_tracking(self, enabled):
        self._start_button.setEnabled(False)
        self._stop_button.setEnabled(False)
        try:
            service_name = '/trajectory_publisher/start'
            try:
                rospy.wait_for_service(service_name, timeout=0.5)
            except rospy.ROSException:
                service_name = '/start'
                rospy.wait_for_service(service_name, timeout=0.5)
            response = rospy.ServiceProxy(service_name, SetBool)(enabled)
            if response.success:
                self._status.setText('Start request accepted' if enabled else 'Stopping / holding position')
            else:
                self._status.setText('Request rejected: {}'.format(response.message))
        except (rospy.ROSException, rospy.ServiceException) as error:
            self._status.setText('Service call failed: {}'.format(error))
        finally:
            self._start_button.setEnabled(True)
            self._stop_button.setEnabled(True)

    def shutdown_plugin(self):
        self._param_widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        self._param_widget.save_settings(plugin_settings, instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._param_widget.restore_settings(plugin_settings, instance_settings)
