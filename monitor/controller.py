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
        self.npc_list = []
    def spawn_vehicle(self, car_name, car_mode) -> Vehicle.Vehicle:
        if not self.vehicles:
            vehicle = Vehicle.Vehicle(self, car_name, mode=car_mode)
        else:
            return None
        self.vehicles.append(vehicle)
        return vehicle

    def spawn_npc(self, number_of_npc=50):
        if self.npc_list:
            return 
        self.spawn_points = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(self.spawn_points)

        if number_of_npc < number_of_spawn_points:
            np.random.shuffle(self.spawn_points)
        elif number_of_npc > number_of_spawn_points:
            number_of_npc = number_of_spawn_points

        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        batch = []
        for n, transform in enumerate(self.spawn_points):
            if n >= number_of_npc:
                break
            blueprint = np.random.choice(blueprints)
            try:
                npc = carla.command.SpawnActor(blueprint, transform)
                batch.append(npc)
                self.spawn_points.pop(0)
            except Exception as e:
                print(e)

        for response in self.client.apply_batch_sync(batch):
            self.npc_list.append(response.actor_id)
            carla.command.SetAutopilot(response.actor_id, True) 
        return 

    def get_vehicle_blueprints(self) -> list:
        if self.vehicle_blueprints:
            for blueprint in self.blueprints.filter('vehicle'):
                self.vehicle_blueprints[blueprint.id] = blueprint
        return self.vehicle_blueprints
