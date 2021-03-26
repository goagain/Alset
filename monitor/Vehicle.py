import time
from KeyboardInput import KeyboardInput
import carla
import numpy as np
import numpy.linalg
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QObject, QEvent, pyqtSignal, pyqtSlot, QTimer
from Ticker import Tickable
import MainApplication

class Vehicle(Tickable):
    MODE_MANUAL = 0,
    MODE_ASSISTANT = 1,
    MODE_AUTOMATIC = 2,

    def __init__(
        self,
        world_controller,
        vehicle_id,
        mode=MODE_AUTOMATIC,
        dashcam=True,
        third_camera=True,
        radar=True,
        lane_invasion_detector=True,
        obstacle_detector=True,
        color=None,
    ):
        super().__init__()

        self.world_controller = world_controller
        self.world = self.world_controller.world

        self.vehicle_controller = None

        self.blueprint = self.world_controller.world.get_blueprint_library(
        ).find(vehicle_id)
        self.monitors = {}

        self.blueprint.set_attribute('role_name', 'trainer')

        if color is None:
            color = np.random.choice(
                self.blueprint.get_attribute('color').recommended_values)
        self.blueprint.set_attribute('color', color)

        bscussed = self.spawn_car()
        count = 0
        while (bscussed == False and count<3):
            bscussed = self.spawn_car()
            count += 1
        
        if( bscussed == False ):
            print('Spawn car failed! please check the connection with Carla Server')

        self.dash_camera = None
        if dashcam:
            self.init_dashcam()

        self.third_camera = None
        if third_camera:
            self.init_thirdcam()

        if obstacle_detector:
            self.init_obstacle_detector()
            self.frontDistance = 100.0
            self.last_distance_update_time = time.time()

        self.lane_invasion_detector = None
        if lane_invasion_detector:
            li_bp = self.world.get_blueprint_library().find(
                'sensor.other.lane_invasion')
            self.lane_invasion_detector = self.world.spawn_actor(
                li_bp, carla.Transform(carla.Location(z=1)), self.entity,
                carla.AttachmentType.Rigid)
            self.lane_invasion_detector.listen(self.on_line_invasion)

        self.radar = None
        if radar:
            radar_bp = self.world.get_blueprint_library().find(
                'sensor.other.radar')
            self.radar = self.world.spawn_actor(
                radar_bp, carla.Transform(carla.Location(z=1)), self.entity,
                carla.AttachmentType.Rigid)

        self.input_controller = KeyboardInput(self.world_controller.uiroot)

        self.mode = mode

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        self.input_controller.reset()

        self._mode = value
        if value == Vehicle.MODE_MANUAL:
            self.entity.set_autopilot(False)
        elif value == Vehicle.MODE_ASSISTANT:
            self.entity.set_autopilot(False)
        elif value == Vehicle.MODE_AUTOMATIC:
            self.entity.set_autopilot(True)

    def spawn_car(self):
        recommend_spawn_points = self.world.get_map().get_spawn_points()
        vehicle_spawn_point = np.random.choice(recommend_spawn_points)

        try:
            self.entity = self.world.spawn_actor(self.blueprint,
                                                 vehicle_spawn_point)
        except Exception as e:
            return False

        return True

    def init_dashcam(self):
        camera_bp = self.world.get_blueprint_library().find(
            'sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(512))
        camera_bp.set_attribute('image_size_y', str(256))

        self.dash_camera = self.world.spawn_actor(
            camera_bp,
            carla.Transform(carla.Location(x=1.5, y=0, z=1.2)),
            self.entity,
            carla.AttachmentType.Rigid,
        )
        # self.dash_camera.set_attribute('role_name', 'dashcam')
        self.dash_camera.listen(self.on_dash_cam)

    def init_thirdcam(self):
        camera_bp = self.world.get_blueprint_library().find(
            'sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(512))
        camera_bp.set_attribute('image_size_y', str(256))

        self.third_cam = self.world.spawn_actor(
            camera_bp,
            carla.Transform(carla.Location(x=-5.5, z=2.5),
                            carla.Rotation(pitch=8.0)), self.entity,
            carla.AttachmentType.SpringArm)
        # self.third_cam.set_attribute('role_name', 'thirdcam')
        self.third_cam.listen(self.on_third_cam)

    def init_obstacle_detector(self):
        od_bp = self.world.get_blueprint_library().find(
            'sensor.other.obstacle')
        od_bp.set_attribute('distance', '100')
        od_bp.set_attribute('only_dynamics', 'false')
        self.obstacle_detector = self.world.spawn_actor(
            od_bp, carla.Transform(carla.Location(z=1)), self.entity,
            carla.AttachmentType.Rigid)
        self.obstacle_detector.listen(self.on_obstacle_detector)

    @property
    def speed(self):
        velocity = self.entity.get_velocity()
        return np.linalg.norm([velocity.x, velocity.y, velocity.z])

    def on_dash_cam(self, image):
        self.monitors['dash_cam'] = image

    def on_third_cam(self, image):
        self.monitors['third_cam'] = image

    def on_line_invasion(self, event):
        """On invasion method"""
        self.monitors['line_invasion'] = event

    @property
    def frontDistance(self):
        if self.last_distance_update_time is not None and time.time() - self.last_distance_update_time < 1:
            return self._frontDistance
        return 1000

    @frontDistance.setter
    def frontDistance(self, value):
        self.last_distance_update_time = time.time()
        self._frontDistance = value

    def on_obstacle_detector(self, event):
        # print(event, f"distance={event.distance} m")
        self.monitors['od'] = event
        self.od_data = event
        self.frontDistance = event.distance

    @property
    def od_data(self):
        try:
            if time.time() - self.last_od_update_time > 1:
                return None
            else:
                return self._od_data
        finally:
            return None
            
    @od_data.setter
    def od_data(self, value):
        self._od_data = value
        self.last_od_update_time = time.time()

    def get_od_data(self, clear):
        if 'od' in self.monitors:
            ret = self.monitors['od']
            if clear:
                self.monitors['od'] = None
            return ret

    def get_line_invasion_data(self, clear):
        if 'line_invasion' in self.monitors:
            ret = self.monitors['line_invasion']
            if clear:
                self.monitors['line_invasion'] = None
            return ret

    @property
    def dash_cam_image(self):
        if 'dash_cam' in self.monitors:
            return self.monitors['dash_cam']

    @property
    def third_cam_image(self):
        if 'third_cam' in self.monitors:
            return self.monitors['third_cam']

    def on_tick(self):
        if not self.entity:
            return
        if self.mode == Vehicle.MODE_MANUAL:
            # print(KeyboardInput.controll)
            self.entity.apply_control(self.input_controller.control)
        if self.mode == Vehicle.MODE_ASSISTANT:
            control = self.input_controller.control
            speedlimit = self.speed_limit / 3.6

            if self.speed < speedlimit:
                control.throttle = max(1,
                                       0.2 + (speedlimit - self.speed) * 0.1)
            else:
                control.throttle = min(0,
                                       0.2 - (self.speed - speedlimit) * 0.02)
            self.entity.apply_control(control)

    def destroy(self):
        if self.dash_camera:
            self.dash_camera.destroy()
            self.dash_camera = None
        if self.third_camera:
            self.third_camera.destroy()
            self.third_camera = None
        if self.obstacle_detector:
            self.obstacle_detector.destroy()
            self.obstacle_detector = None
        if self.radar:
            self.radar.destroy()
            self.radar = None
        if self.lane_invasion_detector:
            self.lane_invasion_detector.destroy()
            self.lane_invasion_detector = None

        #have problem to release this
        #if self.entity:
        #    self.entity.destroy()
        #    self.entity = None
