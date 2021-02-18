from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import QImage, QKeyEvent, QPixmap
from PyQt5.QtCore import QObject, QEvent, pyqtSignal, pyqtSlot, QTimer, Qt
from Vehicle import Vehicle
from controller import Controller
from KeyboardInput import KeyboardInput
import weakref
from logger import log
import Ticker
import yolo_detector

class MainApplication(QMainWindow):
    signal = pyqtSignal(object, QLabel)

    def __init__(self, parent=None):
        super(MainApplication, self).__init__(parent)
        loadUi("./res/UI/MainApplication.ui", self)
        self.initUI()
        self.controller = Controller()
        # self.scene = QGraphicsScene(self)
        Ticker.TickerManager.init(self)
        self.keyboard_input = KeyboardInput(self)
        self.yolo_detector = yolo_detector.yolo_detector()

    def initUI(self):
        self.spawnVehicle.clicked.connect(self.on_click_spawn)
        self.spawnVehicle.installEventFilter(self)
        self.dash_cam.installEventFilter(self)
        self.third_cam.installEventFilter(self)
        self.qimage = QImage()
        self.signal.connect(self.on_render)
        self.vehicle = None
        self.ticker = QTimer(self)
        self.ticker.start(10)
        self.ticker.timeout.connect(self.on_ui_tick)
        self.ticker.timeout.connect(self.on_tick)

        self.assistantModeButton.toggled.connect(self.onSelectAssistantMode)
        self.autoModeButton.toggled.connect(self.onSelectAutoMode)
        self.manualControlButton.toggled.connect(self.onSelectManualMode)

    @property
    def vehicle_mode(self):
        if self.manualControlButton.toggled:
            return Vehicle.MODE_MANUAL
        elif self.assistantModeButton.toggled:
            return Vehicle.MODE_ASSISTANT
        elif self.autoModeButton.toggled:
            return Vehicle.MODE_AUTOMATIC

    def on_click_spawn(self):
        self.vehicle = self.controller.spawn_vehicle("vehicle.tesla.model3",
                                                     self.vehicle_mode)

    @log
    def on_dashcam(self, image):
        if self:
            self.signal.emit(image, self.dash_cam)

    @log
    def on_thirdcam(self, image):
        if self:
            self.signal.emit(image, self.third_cam)

    @log
    def on_radar(self, image):
        if self:
            self.signal.emit(image, self.radar)

    @pyqtSlot(object, QLabel)
    def on_render(self, image, label):
        qimage = QImage(image.raw_data, image.width, image.height,
                        QImage.Format_ARGB32)
        pixmap = QPixmap.fromImage(qimage)
        label.setPixmap(pixmap)

        # self.renderSignal.emit(scene)
    @pyqtSlot(object, QLabel)
    def on_render_detect(self, img_np, width, height, label):
        qimage = QImage(img_np, img_np.shape[1], img_np.shape[0],                                                                                                                                                 
                        QImage.Format_RGB888)                                                                                                                                                                 
        pixmap = QPixmap(qimage)                                                                                                                                                                               
        pixmap = pixmap.scaled(width,height, Qt.IgnoreAspectRatio)                                                                                                                                                    
        label.setPixmap(pixmap)

    def on_tick(self):
        self.keyboard_input.on_tick()

    @log
    def onSelectAssistantMode(self, toggled):
        if toggled and self.vehicle:
            self.vehicle.mode = Vehicle.MODE_ASSISTANT

    @log
    def onSelectAutoMode(self, toggled):
        if toggled and self.vehicle:
            self.vehicle.mode = Vehicle.MODE_AUTOMATIC

    @log
    def onSelectManualMode(self, toggled):
        if toggled and self.vehicle:
            self.vehicle.mode = Vehicle.MODE_MANUAL

    def on_ui_tick(self):
        if not self.vehicle:
            return
        info = []
        info.append(f'Speed: {self.vehicle.speed * 3.6 :.1f} km/h')
        if self.vehicle.dash_cam_image:
            self.on_render(self.vehicle.dash_cam_image, self.dash_cam)
            detect_img = self.yolo_detector.detect(self.vehicle.dash_cam_image)
            self.on_render_detect(detect_img, self.vehicle.dash_cam_image.width, self.vehicle.dash_cam_image.height, self.radar)
        if self.vehicle.third_cam_image:
            self.on_render(self.vehicle.third_cam_image, self.third_cam)
        if self.vehicle.obstacle_detector:
            od_data = self.vehicle.get_od_data(clear=True)
            if od_data:
                info.append(
                    f'actor = {od_data.other_actor.id}, distance = {od_data.distance:.2f} m'
                )

        self.label_info.setText('\n'.join(info))
