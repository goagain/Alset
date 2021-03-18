import carla
import numpy as np
import Vehicle
from numpy import random
import logging
from carla import VehicleLightState as vls


class Controller:
    globalController = None

    def __init__(self, uiroot):
        self.client = None
        self.world = None

        self.uiroot = uiroot
        self.vehicle_blueprints = []
        self.current_vehicle = None
        self.npc_list = []
        self.walker_list = []

    def connect(self, host, port):
        self.client = carla.Client(host, int(port))
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

    def get_maps(self):
        return self.client.get_available_maps()

    def set_map(self, map_name):
        #destory all actors first
        self.destroy()
        self.world = self.client.load_world(map_name)

    def spawn_vehicle(self, car_name, car_mode) -> Vehicle.Vehicle:
        if self.current_vehicle:
             #destory all actors which attched on the current_vehicle
             self.current_vehicle.destroy()
        self.current_vehicle = Vehicle.Vehicle(self, car_name, mode=car_mode)
        return self.current_vehicle

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
                batch.append(npc.then(carla.command.SetAutopilot(carla.command.FutureActor, True)))
                self.spawn_points.pop(0)
                self.world.wait_for_tick()
            except Exception as e:
                print(e)
        self.world.wait_for_tick()
        results = self.client.apply_batch_sync(batch, True)
        for response in self.client.apply_batch_sync(batch):
            self.npc_list.append(response.actor_id)
        self.world.wait_for_tick()

    def spawn_walker(self, number_of_walker=50):
        if self.walker_list:
            return 

        percentagePedestriansRunning = 0.0      # how many pedestrians will run
        percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        blueprintsWalkers = self.world.get_blueprint_library().filter('walker.*')
        spawn_points = []
        for i in range(number_of_walker):
            spawn_point = carla.Transform()
            loc = self.world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = np.random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (np.random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                print(f"spawn walker {i} error:", results[i].error)
            else:
                self.walker_list.append(results[i].actor_id)
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(self.walker_list)):
            batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), self.walker_list[i]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                print(f"mange walker {i} error:", results[i].error)
            else:
                self.walker_list.append(results[i].actor_id)
        
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        self.world.wait_for_tick()
        all_actors = self.world.get_actors(self.walker_list)

        self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(int((len(all_actors)+1)/2),len(all_actors)):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[i-int((len(all_actors)+1)/2)]))
        self.world.wait_for_tick()

    def get_vehicle_blueprints(self) -> list:
        if not self.vehicle_blueprints:
            for blueprint in self.world.get_blueprint_library().filter('vehicle.*'):
                self.vehicle_blueprints.append(blueprint.id)
        return self.vehicle_blueprints

    def destroy(self):
        print('destory actors')

        # if self.world:
        #     self.client.apply_batch([carla.command.DestroyActor(x) for x in self.world.get_actors().filter('*.*')])
        #     self.world = None

        if self.npc_list:
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.npc_list])
            for item in self.npc_list:
                self.npc_list.remove(item)  
            self.npc_list = []

        if self.walker_list:
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.walker_list])
            for item in self.walker_list:
                self.walker_list.remove(item) 
            self.walker_list = []

        if self.current_vehicle:
            self.current_vehicle.destroy()
            self.current_vehicle = None

