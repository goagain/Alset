import time
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import QImage, QKeyEvent, QPixmap, QFont
from PyQt5.QtCore import QObject, QEvent, pyqtSignal, pyqtSlot, QTimer, Qt
from Vehicle import Vehicle
from controller import Controller
from KeyboardInput import KeyboardInput
import weakref
from logger import log
import Ticker
import yolo_detector
import lane_detector
import sys
import traceback


class MainApplication(QMainWindow):
    signal = pyqtSignal(object, QLabel)

    def __init__(self, parent=None):
        super(MainApplication, self).__init__(parent)
        loadUi("./res/UI/MainApplication.ui", self)
        self.initUI()
        self.controller = Controller(self)
        self.statecount = 0
        # self.scene = QGraphicsScene(self)
        Ticker.TickerManager.init(self)

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
        self.detect_limitspeed = 50
        self.ticker = QTimer(self)
        self.ticker.start(10)
        self.ticker.timeout.connect(self.on_ui_tick)
        self.ticker.timeout.connect(self.on_tick)

        self.connectButton.clicked.connect(self.on_click_connect)
        self.loadmapButton.clicked.connect(self.on_click_loadmap)

        self.assistantModeButton.toggled.connect(self.onSelectAssistantMode)
        self.autoModeButton.toggled.connect(self.onSelectAutoMode)
        self.manualControlButton.toggled.connect(self.onSelectManualMode)

        #display speedLimit
        self.speedLimit.setVisible(False)
        self.label_speedlimit.setVisible(False)

        #self.loadmapButton.hide()
        self.label_info.setFont(QFont("Roman times", 10, QFont.Bold))

        self.horizontalLayout_2.setStretchFactor(self.graphics, 2)
        self.horizontalLayout_2.setStretchFactor(self.verticalLayout, 1)

        # center window
        screen = QDesktopWidget().screenGeometry()
        size = self.geometry()
        newleft = (screen.width() - size.width()) / 2
        newTop = (screen.height() - size.height()) / 2
        self.move(newleft, newTop)

    def on_click_connect(self):
        self.controller.connect(self.hostText.text(), self.portText.text())
        self.sceneComboBox.clear()
        self.sceneComboBox.addItems(self.controller.get_maps())

        self.vehicleComboBox.clear()
        self.vehicleComboBox.addItems(self.controller.get_vehicle_blueprints())

        self.yolo_detector = yolo_detector.yolo_detector()

    def on_click_loadmap(self):
        self.controller.set_map(self.sceneComboBox.currentText())

    @property
    def vehicle_mode(self):
        if self.manualControlButton.isChecked():
            return Vehicle.MODE_MANUAL
        elif self.assistantModeButton.isChecked():
            return Vehicle.MODE_ASSISTANT
        elif self.autoModeButton.isChecked():
            return Vehicle.MODE_AUTOMATIC

    def on_click_spawn(self):
        self.vehicle = self.controller.spawn_vehicle(
            self.vehicleComboBox.currentText(), self.vehicle_mode)

    def on_click_spawn_npc(self):
        self.controller.spawn_npc()
        self.controller.spawn_walker()

    def closeEvent(self, event):
        self.controller.destroy()
        event.accept()

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
        pixmap = pixmap.scaled(width, height, Qt.IgnoreAspectRatio)
        label.setPixmap(pixmap)

    @pyqtSlot(object, QLabel)
    def on_render_laneview(self, img_np, width, height, label):
        qimage = QImage(img_np, img_np.shape[1], img_np.shape[0],
                        QImage.Format_RGB888)
        pixmap = QPixmap(qimage)
        pixmap = pixmap.scaled(width, height, Qt.IgnoreAspectRatio)
        label.setPixmap(pixmap)

    def on_tick(self):
        if self.vehicle:
            self.vehicle.speed_limit = self.speedLimit.value()

    @log
    def onSelectAssistantMode(self, toggled):
        if toggled and self.vehicle:
            self.vehicle.mode = Vehicle.MODE_ASSISTANT
            self.speedLimit.setVisible(True)
            self.label_speedlimit.setVisible(True)

    @log
    def onSelectAutoMode(self, toggled):
        if toggled and self.vehicle:
            self.vehicle.mode = Vehicle.MODE_AUTOMATIC
            self.speedLimit.setVisible(False)
            self.label_speedlimit.setVisible(False)

    @log
    def onSelectManualMode(self, toggled):
        if toggled and self.vehicle:
            self.vehicle.mode = Vehicle.MODE_MANUAL
            self.speedLimit.setVisible(False)
            self.label_speedlimit.setVisible(False)

    def on_ui_tick(self):
        if not self.vehicle:
            return

        info = []

        if self.vehicle.third_cam_image:
            self.on_render(self.vehicle.third_cam_image, self.third_cam)
        if self.vehicle.obstacle_detector:
            od_data = self.vehicle.od_data
            if od_data:
                info.append(
                    f'actor = {od_data.other_actor.id}, distance = {od_data.distance:.2f} m'
                )
            else:
                info.append('') # placeholder

            frontDistance = self.vehicle.frontDistance
            if frontDistance > 100:
                self.labelDistance.setText(f'>100 m')
            else:
                self.labelDistance.setText(f'{frontDistance:.2f} m')
            if frontDistance > 50:  # safe distance, green
                self.labelDistance.setStyleSheet("background-color:green;")
            elif frontDistance > 20:  # warning distance, yellow
                self.labelDistance.setStyleSheet("background-color:yellow;")
            else:  # blink red effect
                if (time.time() // 0.5) % 2 == 0:
                    self.labelDistance.setStyleSheet("background-color:red;")
                else:
                    self.labelDistance.setStyleSheet("")

        if self.vehicle.lane_invasion_detector:
            line_invasion_data = self.vehicle.get_line_invasion_data(
                clear=True)
            if line_invasion_data:
                lane_types = set(
                    x.type for x in line_invasion_data.crossed_lane_markings)
                text = ['%r' % str(x).split()[-1] for x in lane_types]
                info.append(f'Crossed line: {text}')

        if self.vehicle.dash_cam_image:
            self.on_render(self.vehicle.dash_cam_image, self.dash_cam)
            detect_img, speed = self.yolo_detector.detect(
                self.vehicle.dash_cam_image)
            self.on_render_detect(detect_img,
                                  self.vehicle.dash_cam_image.width,
                                  self.vehicle.dash_cam_image.height,
                                  self.radar)
            #detect_img1,bchange, speed = self.speedsign_detector.detect(self.vehicle.dash_cam_image)
            if (speed > 0):
                self.detect_limitspeed = speed

            if self.vehicle.speed > 0:
                if ((self.vehicle.speed * 3.6) > self.detect_limitspeed):
                    info.append('Over speed!!!!')
                    # Begin to flash or reduce speed

            try:
                #deviation is the deviation from the center of the road
                detect_img, deviation = lane_detector.process_lane_detect(self.vehicle.dash_cam_image)
                direction = "left" if deviation < 0 else "right"
                self.on_render_laneview(detect_img,
                                        self.vehicle.dash_cam_image.width,
                                        self.vehicle.dash_cam_image.height,
                                        self.laneview)
            except Exception as e:
                traceback.print_exception(*sys.exc_info())
                #self.vehicle.dash_cam_image.save_to_disk(r'c:\tmp\lde.png')
                print(e)
                #raise(e)

        self.speedText.setText(str(int(self.vehicle.speed * 3.6)))

        info.append(f'Speed: {self.vehicle.speed * 3.6 :.1f} km/h')
        info.append(f'Speed Limit: {self.detect_limitspeed} km/h')
        self.label_info.setText('\n'.join(info))
