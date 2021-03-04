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
import lane_detector
import speedsign_detector
import sys
import traceback

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
        self.speedsign_detector = speedsign_detector.speedsign_detector()
        self.lane_detector = lane_detector.lane_detector()

    def initUI(self):
        self.spawnVehicle.clicked.connect(self.on_click_spawn)
        self.spawnVehicle.installEventFilter(self)
        self.spawnNPC.clicked.connect(self.on_click_spawn_npc)
        self.spawnNPC.installEventFilter(self)
        self.dash_cam.installEventFilter(self)
        self.third_cam.installEventFilter(self)
        self.qimage = QImage()
        self.signal.connect(self.on_render)
        self.vehicle = None
        self.curlimitspeed = 50
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

    def on_click_spawn_npc(self):
        self.vehicle = self.controller.spawn_npc()

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

    @log
    def on_laneview(self, image):
        if self:
            self.signal.emit(image, self.laneview)

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

    @pyqtSlot(object, QLabel)
    def on_render_laneview(self, img_np, width, height, label):
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
        
        if self.vehicle.dash_cam_image:
            self.on_render(self.vehicle.dash_cam_image, self.dash_cam)
            detect_img = self.yolo_detector.detect(self.vehicle.dash_cam_image)
            self.on_render_detect(detect_img, self.vehicle.dash_cam_image.width, self.vehicle.dash_cam_image.height, self.radar)
            detect_img1,bchange, speed = self.speedsign_detector.detect(self.vehicle.dash_cam_image)
            if( bchange):
                self.curlimitspeed = speed
            try:
                #detect_img = lane_detector.process_lane_detect(self.vehicle.dash_cam_image)
                detect_img = self.lane_detector.detect_lines(self.vehicle.dash_cam_image)
                self.on_render_laneview(detect_img, self.vehicle.dash_cam_image.width, self.vehicle.dash_cam_image.height, self.laneview)
            except Exception as e:
                traceback.print_exception(*sys.exc_info())
                #self.vehicle.dash_cam_image.save_to_disk(r'c:\tmp\lde.png')
                print(e)
                #raise(e)
        if self.vehicle.third_cam_image:
            self.on_render(self.vehicle.third_cam_image, self.third_cam)
        if self.vehicle.obstacle_detector:
            od_data = self.vehicle.get_od_data(clear=True)
            if od_data:
                info.append(
                    f'actor = {od_data.other_actor.id}, distance = {od_data.distance:.2f} m'
                )

     
        info.append(f'Speed: {self.vehicle.speed * 3.6 :.1f} km/h')
        info.append(f'Speed Limit: {self.curlimitspeed} km/h')
        self.label_info.setText('\n'.join(info))
