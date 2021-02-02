from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import pyqtSignal, pyqtSlot, QTimer
from controller import Controller
import weakref

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
        self.ticker = QTimer(self)
        self.ticker.start(10)
        self.ticker.timeout.connect(self.on_tick)

    def initUI(self):
        self.spawnVehicle.clicked.connect(self.on_click_spawn)

    def on_click_spawn(self):
        self.vehicle = self.controller.spawn_vehicle("vehicle.tesla.model3")

    def on_dashcam(self, image):
        if self:
            self.signal.emit(image, self.dash_cam)

    def on_thirdcam(self, image):
        if self:
            self.signal.emit(image, self.third_cam)

    def on_radar(self, image):
        if self:
            self.signal.emit(image, self.radar)

    @pyqtSlot(object, QLabel)
    def on_render(self, image, label):
        qimage = QImage(image.raw_data, image.width,
                        image.height, QImage.Format_ARGB32)
        pixmap = QPixmap.fromImage(qimage)
        label.setPixmap(pixmap)

        # self.renderSignal.emit(scene)

    def on_tick(self):
        if not self.vehicle:
            return
        info = []
        info.append(f'Speed: {self.vehicle.speed * 3.6 :.1f} km/h')
        if self.vehicle.dash_cam_image:
            self.on_render(self.vehicle.dash_cam_image, self.dash_cam)
        if self.vehicle.third_cam_image:
            self.on_render(self.vehicle.third_cam_image, self.third_cam)
        if self.vehicle.obstacle_detector:
            od_data = self.vehicle.get_od_data(clear=True)
            if od_data:
                info.append(f'actor = {od_data.other_actor.id}, distance = {od_data.distance:.2f} m')
        
        self.label_info.setText('\n'.join(info))