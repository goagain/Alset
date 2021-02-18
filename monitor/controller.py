import carla
import numpy as np
import Vehicle


class Controller:
    globalController = None

    def __init__(self):
        self.client = carla.Client("localhost", 2000)
        self.client.set_timeout(10.0)

        self.world = self.client.load_world('Town02')

        self.vehicle_blueprints = {}
        self.vehicles = []
    def spawn_vehicle(self, car_name, car_mode) -> Vehicle.Vehicle:
        if not self.vehicles:
            vehicle = Vehicle.Vehicle(self, car_name, mode=car_mode)
        else:
            return None
        self.vehicles.append(vehicle)

        return vehicle

    def get_vehicle_blueprints(self) -> list:
        if self.vehicle_blueprints:
            for blueprint in self.blueprints.filter('vehicle'):
                self.vehicle_blueprints[blueprint.id] = blueprint
        return self.vehicle_blueprints
