import carla
import numpy as np


class Vehicle:
    def __init__(self, controller, vehicle_id, auto_pilot=True, dashcam=True, third_camera=True, color=None):
        self.controller = controller
        self.world = self.controller.world
        self.blueprint = self.controller.world.get_blueprint_library().find(vehicle_id)

        recommend_spawn_points = self.world.get_map().get_spawn_points()
        vehicle_spawn_point = np.random.choice(recommend_spawn_points)

        if color is None:
            color = np.random.choice(
                self.blueprint.get_attribute('color').recommended_values)
        self.blueprint.set_attribute('color', color)

        self.entity = self.world.spawn_actor(
            self.blueprint, vehicle_spawn_point)

        self.set_autopilot(auto_pilot)

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(1028))
        camera_bp.set_attribute('image_size_y', str(720))

        self.dash_camera = None
        if dashcam:
            self.dash_camera = self.world.spawn_actor(camera_bp,
                                                      carla.Transform(carla.Location(
                                                          x=1.5, y=0, z=1.2)),

                                                      self.entity,
                                                      carla.AttachmentType.Rigid
                                                      )

        self.third_camera = None
        if third_camera:
            self.third_camera = self.world.spawn_actor(camera_bp,
                                                       carla.Transform(carla.Location(
                                                           x=-5.5, z=2.5), carla.Rotation(pitch=8.0)),
                                                       self.entity,
                                                       carla.AttachmentType.SpringArm
                                                       )

    def set_autopilot(self, value):
        self.entity.set_autopilot = value
        pass
