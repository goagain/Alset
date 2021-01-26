from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot
from controller import Controller


class MainApplication(QMainWindow):
    signal = pyqtSignal(object, QLabel)

    def __init__(self, parent=None):
        super(MainApplication, self).__init__(parent)
        loadUi("./res/UI/MainApplication.ui", self)
        self.initUI()
        self.controller = Controller()
        # self.scene = QGraphicsScene(self)

        self.qimage = QImage()
        self.signal.connect(self.on_render)
        self.vehicle = None

    def initUI(self):
        self.spawnVehicle.clicked.connect(self.on_click_spawn)

    def on_click_spawn(self):
        self.vehicle = self.controller.spawn_vehicle("vehicle.tesla.model3")
        if self.vehicle.third_camera:
            self.vehicle.third_camera.listen(self.on_thirdcam)
        # self.vehicle.dash_camera.listen(self.on_dashcam)

    def on_dashcam(self, image):
        self.signal.emit(image, self.dash_cam)

    def on_thirdcam(self, image):
        self.signal.emit(image, self.third_cam)

    @pyqtSlot(object, QLabel)
    def on_render(self, image, label):
        qimage = QImage(image.raw_data, image.width,
                        image.height, QImage.Format_ARGB32)
        pixmap = QPixmap.fromImage(qimage)
        label.setPixmap(pixmap)

        # self.renderSignal.emit(scene)
