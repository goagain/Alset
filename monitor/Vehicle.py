import carla
import numpy as np
import numpy.linalg


class Vehicle:
    def __init__(self,
                 controller,
                 vehicle_id,
                 auto_pilot=True,
                 dashcam=True,
                 third_camera=True,
                 radar=True,
                 lane_invasion_detector=True,
                 obstacle_detector=True,
                 color=None):
        self.controller = controller
        self.world = self.controller.world
        self.blueprint = self.controller.world.get_blueprint_library().find(
            vehicle_id)

        self.monitors = {}

        recommend_spawn_points = self.world.get_map().get_spawn_points()
        vehicle_spawn_point = np.random.choice(recommend_spawn_points)

        self.blueprint.set_attribute('role_name', 'trainer')

        if color is None:
            color = np.random.choice(
                self.blueprint.get_attribute('color').recommended_values)
        self.blueprint.set_attribute('color', color)

        self.entity = self.world.spawn_actor(self.blueprint,
                                             vehicle_spawn_point)

        self.set_autopilot(auto_pilot)

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

        self.radar = None
        if radar:
            radar_bp = self.world.get_blueprint_library().find(
                'sensor.other.radar')
            self.radar = self.world.spawn_actor(
                radar_bp, carla.Transform(carla.Location(z=1)), self.entity,
                carla.AttachmentType.Rigid)

    def set_autopilot(self, value):
        self.entity.set_autopilot(value)
        pass

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

    @property
    def speed(self):
        velocity = self.entity.get_velocity()
        return np.linalg.norm([velocity.x, velocity.y, velocity.z])

    def on_dash_cam(self, image):
        self.monitors['dash_cam'] = image

    def on_third_cam(self, image):
        self.monitors['third_cam'] = image
    
    def on_obstacle_detector(self, event):
        # print(event, f"distance={event.distance} m")
        self.monitors['od'] = event

    def get_od_data(self, clear):
        if 'od' in self.monitors:
            ret = self.monitors['od']
            if clear:
                self.monitors['od'] = None
            return ret
    @property
    def dash_cam_image(self):
        if 'dash_cam' in self.monitors:
            return self.monitors['dash_cam']

    @property
    def third_cam_image(self):
        if 'third_cam' in self.monitors:
            return self.monitors['third_cam']