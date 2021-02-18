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
            KeyboardInput.controll = carla.VehicleControl()

    def on_tick(self):
        pass

    def eventFilter(self, target: 'QObject', event: 'QEvent') -> bool:
        if event.type() == QEvent.KeyPress or event.type() == QEvent.KeyRelease:
            key = event.key()
            isPress = event.type() == QEvent.KeyPress
            if key == Qt.Key_W:
                KeyboardInput.controll.throttle = isPress
            elif key == Qt.Key_S:
                KeyboardInput.controll.brake = isPress
            elif key == Qt.Key_A:
                KeyboardInput.controll.steer = -1.0 * isPress
            elif key == Qt.Key_D:
                KeyboardInput.controll.steer = 1.0 * isPress
            elif key == Qt.Key_Space: # Space
                KeyboardInput.controll.hand_brake = isPress
            elif key == Qt.Key_BracketLeft and isPress:
                KeyboardInput.controll.reverse = True
            elif key == Qt.Key_BracketRight and isPress:
                KeyboardInput.controll.reverse = False

            return True
        return super().eventFilter(target, event)
