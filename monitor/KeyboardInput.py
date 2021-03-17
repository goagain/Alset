from PyQt5.QtCore import QObject, QEvent, Qt
import carla

class KeyboardInput(QObject):
    def __init__(self, parent: 'QObject') -> None:
        super().__init__(parent=parent)
        parent.installEventFilter(self)
        self.delta = 0.05

        self.reset(True)

    def reset(self, init = False):
        if init:
            self.control = carla.VehicleControl()


    def eventFilter(self, target: 'QObject', event: 'QEvent') -> bool:
        if event.type() == QEvent.KeyPress or event.type() == QEvent.KeyRelease:
            key = event.key()
            isPress = event.type() == QEvent.KeyPress
            if key == Qt.Key_W:
                self.control.throttle = isPress
            elif key == Qt.Key_S:
                self.control.brake = isPress
            elif key == Qt.Key_A:
                self.control.steer = -1.0 * isPress
            elif key == Qt.Key_D:
                self.control.steer = 1.0 * isPress
            elif key == Qt.Key_Space: # Space
                self.control.hand_brake = isPress
            elif key == Qt.Key_BracketLeft and isPress:
                self.control.reverse = True
            elif key == Qt.Key_BracketRight and isPress:
                self.control.reverse = False

            return True
        return super().eventFilter(target, event)
