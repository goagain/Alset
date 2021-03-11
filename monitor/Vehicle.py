from KeyboardInput import KeyboardInput
import carla
import numpy as np
import numpy.linalg
from PyQt5.QtWidgets import *
from PyQt5.uic import loadUi
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QObject, QEvent, pyqtSignal, pyqtSlot, QTimer
from Ticker import Tickable
from logger import log
from dataexport import *

OUTPUT_FOLDER = '_out'

def maybe_create_dir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


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

        # collect data or not, default is False
        self.b_savedata = False
        if( self.b_savedata ):
           self.init_data_dir(OUTPUT_FOLDER)

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
        while( bscussed == False ):
            bscussed = self.spawn_car()


        self.mode = mode

        self.dash_camera = None
        if dashcam:
            self.init_dashcam()

        self.third_camera = None
        if third_camera:
            self.init_thirdcam()

        if obstacle_detector:
            self.init_obstacle_detector()

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

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        self._mode = value
        if value == Vehicle.MODE_MANUAL:
            self.entity.set_autopilot(False)
        elif value == Vehicle.MODE_ASSISTANT:
            self.entity.set_autopilot(True)
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
        od_bp.set_attribute('distance', '50')
        od_bp.set_attribute('only_dynamics', 'true')
        self.obstacle_detector = self.world.spawn_actor(
            od_bp, carla.Transform(carla.Location(z=1)), self.entity,
            carla.AttachmentType.Rigid)
        self.obstacle_detector.listen(self.on_obstacle_detector)

    def init_data_dir(self,dirpath):  
        self.third_cam_dir = os.path.join(OUTPUT_FOLDER, 'third_cam')
        self.dash_cam_dir = os.path.join(OUTPUT_FOLDER, 'dash_cam')
        self.obstacle_detector_dir = os.path.join(OUTPUT_FOLDER,'obstacle_detector')
        self.velocity_dir = os.path.join(OUTPUT_FOLDER, 'velocity')

        maybe_create_dir(self.third_cam_dir)
        maybe_create_dir(self.dash_cam_dir)
        maybe_create_dir(self.obstacle_detector_dir)
        maybe_create_dir(self.velocity_dir)

    @property
    def speed(self):
        velocity = self.entity.get_velocity()

        if( self.b_savedata):
            current_speed = np.linalg.norm([velocity.x, velocity.y, velocity.z])
            velocity_dict = {
                'x':velocity.x,
                'y':velocity.y,
                'z':velocity.z
                }
            save_velocity_data(f'{self.velocity_dir}/speed_history.txt',velocity_dict)
        return np.linalg.norm([velocity.x, velocity.y, velocity.z])

    def on_dash_cam(self, image):
        self.monitors['dash_cam'] = image
        if( self.b_savedata ):
            #cc = carla.ColorConverter.LogarithmicDepth
            cc = carla.ColorConverter.Raw
            save_image_data(f'{self.dash_cam_dir}/{image.frame:06d}.png',image, cc)

    def on_third_cam(self, image):
        self.monitors['third_cam'] = image

        if( self.b_savedata):
            #cc = carla.ColorConverter.LogarithmicDepth
            cc = carla.ColorConverter.Raw
            save_image_data(f'{self.third_cam_dir}/{image.frame:06d}.png',image, cc)

    def on_line_invasion(self, event):
        """On invasion method"""
        self.monitors['line_invasion'] = event
        

    def on_obstacle_detector(self, event):
        # print(event, f"distance={event.distance} m")
        self.monitors['od'] = event

        if( self.b_savedata):
            obstacle_detector_dict = {
                'frame':event.frame,
                'timestamp':event.timestamp,
                'distance':event.distance
                }

            #save to separate file
            pth = f'{self.obstacle_detector_dir}/{event.frame:06d}.txt'
            save_obstacle_detector_data(pth,obstacle_detector_dict)
            #save to the same file
            #save_to_disk(f'{self.obstacle_detector_dir}/event.txt',obstacle_detector_dict)
            

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
        if self.mode == Vehicle.MODE_MANUAL:
            # print(KeyboardInput.controll)
            self.entity.apply_control(KeyboardInput.controll)