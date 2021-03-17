import carla
import numpy as np
import Vehicle
from numpy import random
import logging
from carla import VehicleLightState as vls


class Controller:
    globalController = None

    def __init__(self):
        self.client = None
        self.world = None

        self.vehicle_blueprints = []
        self.current_vehicle = None
        self.npc_list = []

    def connect(self, host, port):
        self.client = carla.Client(host, int(port))
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

    def get_maps(self):
        return self.client.get_available_maps()

    def set_map(self, map_name):
        self.world = self.client.load_world(map_name)




    def spawn_vehicle(self, car_name, car_mode) -> Vehicle.Vehicle:
        self.current_vehicle = Vehicle.Vehicle(self, car_name, mode=car_mode)
        return self.current_vehicle

    def spawn_npc(self, number_of_vehicles = 10, number_of_walkers=50):
        world = self.world
        client = self.client

        vehicles_list = []
        walkers_list = []
        all_id = []
        client.set_timeout(10.0)
        synchronous_master = False
        try:
            world = client.get_world()

            traffic_manager = client.get_trafficmanager(8000)
            traffic_manager.set_global_distance_to_leading_vehicle(1.0)

            blueprints = world.get_blueprint_library().filter('vehicle.*')
            blueprintsWalkers = world.get_blueprint_library().filter('walker.pedestrian.*')
            blueprints = sorted(blueprints, key=lambda bp: bp.id)

            spawn_points = world.get_map().get_spawn_points()
            number_of_spawn_points = len(spawn_points)

            if number_of_vehicles < number_of_spawn_points:
                random.shuffle(spawn_points)
            elif number_of_vehicles > number_of_spawn_points:
                msg = 'requested %d vehicles, but could only find %d spawn points'
                logging.warning(msg, number_of_vehicles, number_of_spawn_points)
                number_of_vehicles = number_of_spawn_points

            # @todo cannot import these directly.
            SpawnActor = carla.command.SpawnActor
            SetAutopilot = carla.command.SetAutopilot
            SetVehicleLightState = carla.command.SetVehicleLightState
            FutureActor = carla.command.FutureActor

            # --------------
            # Spawn vehicles
            # --------------
            batch = []
            for n, transform in enumerate(spawn_points):
                if n >= number_of_vehicles:
                    break
                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)
                blueprint.set_attribute('role_name', 'autopilot')

                # prepare the light state of the cars to spawn
                light_state = vls.NONE

                # spawn the cars and set their autopilot and light state all together
                batch.append(SpawnActor(blueprint, transform)
                    .then(SetAutopilot(FutureActor, True, traffic_manager.get_port()))
                    .then(SetVehicleLightState(FutureActor, light_state)))

            for response in client.apply_batch_sync(batch, synchronous_master):
                if response.error:
                    logging.error(response.error)
                else:
                    vehicles_list.append(response.actor_id)

            # -------------
            # Spawn Walkers
            # -------------
            # some settings
            percentagePedestriansRunning = 0.0      # how many pedestrians will run
            percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
            # 1. take all the random locations to spawn
            spawn_points = []
            for i in range(number_of_walkers):
                spawn_point = carla.Transform()
                loc = world.get_random_location_from_navigation()
                if (loc != None):
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
            # 2. we spawn the walker object
            batch = []
            walker_speed = []
            for spawn_point in spawn_points:
                walker_bp = random.choice(blueprintsWalkers)
                # set as not invincible
                if walker_bp.has_attribute('is_invincible'):
                    walker_bp.set_attribute('is_invincible', 'false')
                # set the max speed
                if walker_bp.has_attribute('speed'):
                    if (random.random() > percentagePedestriansRunning):
                        # walking
                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                    else:
                        # running
                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
                else:
                    print("Walker has no speed")
                    walker_speed.append(0.0)
                batch.append(SpawnActor(walker_bp, spawn_point))
            results = client.apply_batch_sync(batch, True)
            walker_speed2 = []
            for i in range(len(results)):
                if results[i].error:
                    logging.error(results[i].error)
                else:
                    walkers_list.append({"id": results[i].actor_id})
                    walker_speed2.append(walker_speed[i])
            walker_speed = walker_speed2
            # 3. we spawn the walker controller
            batch = []
            walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
            for i in range(len(walkers_list)):
                batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
            results = client.apply_batch_sync(batch, True)
            for i in range(len(results)):
                if results[i].error:
                    logging.error(results[i].error)
                else:
                    walkers_list[i]["con"] = results[i].actor_id
            # 4. we put altogether the walkers and controllers id to get the objects from their id
            for i in range(len(walkers_list)):
                all_id.append(walkers_list[i]["con"])
                all_id.append(walkers_list[i]["id"])
            all_actors = world.get_actors(all_id)

            # wait for a tick to ensure client receives the last transform of the walkers we have just created
            world.tick()

            # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
            # set how many pedestrians can cross the road
            world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
            for i in range(0, len(all_id), 2):
                # start walker
                all_actors[i].start()
                # set walk to random point
                all_actors[i].go_to_location(world.get_random_location_from_navigation())
                # max speed
                all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

            print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

            # example of how to use parameters
            traffic_manager.global_percentage_speed_difference(30.0)
        finally:
            pass
    def get_vehicle_blueprints(self) -> list:
        if not self.vehicle_blueprints:
            for blueprint in self.world.get_blueprint_library().filter('vehicle.*'):
                self.vehicle_blueprints.append(blueprint.id)
        return self.vehicle_blueprints
