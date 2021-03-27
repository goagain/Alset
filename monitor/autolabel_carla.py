### Program to test the carla_vehicle_annotator.py functions performance
### By Mukhlas Adib
### Based example from CARLA Github client_bounding_boxes.py
### 2020
### Last tested on CARLA 0.9.8

### This program is a modified version of CARLA example program client_bounding_boxes.py
### Except functions that convert 3D bounding boxes to 2D bounding boxes
### CARLA Simulator and client_bounding_boxes.py are licensed under the terms of the MIT license
### For a copy, see <https://opensource.org/licenses/MIT>
### For more information about CARLA Simulator, visit https://carla.org/

"""
An example of client-side bounding boxes with basic car controls.

Controls:

    W            : throttle
    S            : brake
    AD           : steer
    Space        : hand-brake
    Q            : debugging (show the depth image and try to save the output)
    
    ESC          : quit
"""

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import os
import traceback 
import argparse
# ==============================================================================
# -- imports -------------------------------------------------------------------
# ==============================================================================

import carla
import random
import queue

try:
    import pygame
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_SPACE
    from pygame.locals import K_a
    from pygame.locals import K_d
    from pygame.locals import K_s
    from pygame.locals import K_w
    from pygame.locals import K_q
    from pygame.locals import K_p
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')

try:
    import numpy as np
except ImportError:
    raise RuntimeError('cannot import numpy, make sure numpy package is installed')
from datetime import datetime

np.set_printoptions(threshold=200)

VIEW_WIDTH = 1920//2
VIEW_HEIGHT = 1080//2
VIEW_FOV = 90

BB_COLOR = (248, 64, 24)
D_COLOR = (0, 0, 240)

depth_queue = queue.Queue()
camera_queue = queue.Queue()
# ==============================================================================
# -- set filter parameters -----------------------------------------------------
# ==============================================================================

import threading
frame_id1 = 0
frame_id2 = 0
def get_thread_id():
    t = threading.currentThread()
    #print(f'Thread {t.ident} - {t.getName()}')
    return t

max_distance = 40
npc_car_num = 40
npc_walker_num = 40

depth_margin = -1
patch_ratio = 0.5
resize_ratio=0.5
is_save_to_disk=True
city = 'Town01'
weather_id = -1
weathers = [
    carla.WeatherParameters.ClearNoon,
    carla.WeatherParameters.CloudyNoon,
    carla.WeatherParameters.WetNoon,
    carla.WeatherParameters.WetCloudyNoon,
    carla.WeatherParameters.MidRainyNoon,
    carla.WeatherParameters.HardRainNoon,
    carla.WeatherParameters.SoftRainNoon,
    carla.WeatherParameters.ClearSunset,
    carla.WeatherParameters.CloudySunset,
    carla.WeatherParameters.WetSunset,
    carla.WeatherParameters.WetCloudySunset,
    carla.WeatherParameters.MidRainSunset,
    carla.WeatherParameters.HardRainSunset,
    carla.WeatherParameters.SoftRainSunset,
]
starttime = datetime.today().strftime('%Y%m%d%H%M%S')
fileframe = 1
img_folder = r'C:\tmp\val\img'
txt_folder = r'C:\tmp\val\txt'
chk_folder = r'C:\tmp\val\chk'
#print (os.path.join(img_folder, "a.png"))


class TaggedObject():
    def __init__(self, world, box, trans, actor_id = '', tag = {'name':'','index':-1}):
        self.world = world
        self.box = box
        self.trans = trans
        self.actor_id = actor_id
        self.tag = tag

class TaggedObjects():
    def __init__(self, world):
        self.world = world
        self.objects = []
        self.namedict = {
            'vehicle.chevrolet.impala':             'Vehicle',
            'vehicle.mercedesccc.mercedesccc':      'Vehicle',
            'vehicle.audi.a2':                      'Vehicle',
            'vehicle.nissan.micra':                 'Vehicle',
            'vehicle.carlamotors.carlacola':        'Vehicle',
            'vehicle.audi.tt':                      'Vehicle',
            'vehicle.bmw.grandtourer':              'Vehicle',
            'vehicle.harley-davidson.low_rider':    'Motocycle',
            'vehicle.bmw.isetta':                   'Vehicle',
            'vehicle.chargercop2020.chargercop2020':'Vehicle',
            'vehicle.citroen.c3':                   'Vehicle',
            'vehicle.dodge_charger.police':         'Vehicle',
            'vehicle.jeep.wrangler_rubicon':        'Vehicle',
            'vehicle.mercedes-benz.coupe':          'Vehicle',
            'vehicle.mini.cooperst':                'Vehicle',
            'vehicle.nissan.patrol':                'Vehicle',
            'vehicle.seat.leon':                    'Vehicle',
            'vehicle.toyota.prius':                 'Vehicle',
            'vehicle.yamaha.yzf':                   'Motocycle',
            'vehicle.kawasaki.ninja':               'Motocycle',
            'vehicle.bh.crossbike':                 'Bicycle',
            'vehicle.tesla.model3':                 'Vehicle',
            'vehicle.gazelle.omafiets':             'Bicycle',
            'vehicle.tesla.cybertruck':             'Vehicle',
            'vehicle.diamondback.century':          'Bicycle',
            'vehicle.audi.etron':                   'Vehicle',
            'vehicle.volkswagen.t2':                'Vehicle',
            'vehicle.lincoln.mkz2017':              'Vehicle',
            'vehicle.mustang.mustang':              'Vehicle',
            'vehicle.lincoln2020.mkz2020':          'Vehicle',
            'vehicle.charger2020.charger2020':      'Vehicle',
            'TrafficLight':                         'Remove',
            'Vehicles':                             'Vehicle',
            'Pedestrians':                          'Pedestrian',
            'BusStop':                              'BusStop',
            'SpeedLimit':                           'Remove',
            'traffic.speed_limit.30':               'traffic.speed_limit.30',
            'traffic.speed_limit.60':               'traffic.speed_limit.60',
            'traffic.speed_limit.90':               'traffic.speed_limit.90',
            'traffic_light.Red':                    'traffic.light.Red',
            'traffic_light.Green':                  'traffic.light.Green',
            'traffic_light.Yellow':                 'traffic.light.Yellow',
            'traffic.stop':                         'traffic.stop',
            'traffic.speed_limit.40'  :             'traffic.speed_limit.40'
            }
        self.namedict2 = {}
        self.namelist2 = []
        i = 0
        for k, tag in self.namedict.items():
            if tag=='Remove':
                continue
            if tag in self.namedict2:
                continue
            self.namedict2[tag] = i
            self.namelist2.append(tag)
            i += 1
        # namelist2 = ['Vehicle', 'Motocycle', 'Bicycle', 'Pedestrian', 'BusStop', 'traffic.speed_limit.30', 'traffic.speed_limit.60', 'traffic.speed_limit.90', 'traffic.light.Red', 'traffic.light.Green', 'traffic.light.Yellow', 'traffic.stop']
        print(f'total {len(self.namelist2)} categories: {self.namelist2}')
        self._update_fixed_objects()

    def _updated_tags(self, tag, actor_id):
        if actor_id:
            actor = self.world.get_actors([actor_id])[0]
            if actor.type_id.find('traffic_light')>=0:
                tag = 'traffic_light.' + str(actor.get_state())
            elif actor.type_id.find('speed_limit')>=0:
                tag = actor.type_id
        if tag:
            ntag = self.namedict[tag]
            return { 'name': ntag, 'index': self.namedict2[ntag]}

    def _get_actor_id(self, loc):
        actors = self.world.get_actors()
        for a in actors:
            if loc.distance(a.get_transform().location)<0.1:
                return a.id
        return ''

    def _update_fixed_objects(self):
        eo = self.world.get_environment_objects()
        for e in eo:
            tag = ''
            if str(e.type) in ['TrafficLight','Vehicles','Pedestrians']:
                tag = str(e.type)
            elif (str(e.type) in ['Static','Other','NONE']) and (e.name.lower().find('busstop')>=0 or e.name.lower().find('bus_stop')>=0):
                tag = 'BusStop'
            elif e.name.find('SpeedLimit')>=0 and e.bounding_box.extent.z<0.5:
                tag = 'SpeedLimit'
            if not tag:
                continue
            actor_id = self._get_actor_id(e.transform.location)
            box = e.bounding_box
            trans = e.transform
            trans.location = box.location
            trans.rotation = box.rotation
            box.location = carla.Location(0.0,0.0,0.0)
            box.rotation = carla.Rotation(0.0,0.0,0.0)
            to = TaggedObject(self.world, box, trans, actor_id, self._updated_tags(tag,actor_id))
            #print(f'fixed obj id:{e.id} - {tag} type:{e.type}:{e.type=="TrafficLight"}')
            self.objects.append(to)

    def get_all_objects(self):
        objects = self.objects.copy()
        # update tag for traffic light
        for o in objects:
            tag = self._updated_tags('', o.actor_id)
            if tag:
                o.tag = tag

        vehicles = self.world.get_actors().filter('vehicle.*')
        for v in vehicles:
            id = v.id
            name = v.type_id
            box = v.bounding_box
            trans = v.get_transform()
            if hasattr(v, 'attributes') and v.attributes['number_of_wheels']:
                if v.attributes['number_of_wheels']=='2':
                    # some bicyle or motocyle's box data is wrong in Carla, need fix the wrong data.
                    if v.bounding_box.location.y>5:
                        box.location.y = 0.1
                    if v.bounding_box.extent.y<0.01:
                        box.extent.y = 0.2
                    box.extent.z = box.extent.z * 1.25
                    box.location.z = box.location.z * 1.25
            to = TaggedObject(self.world, box, trans, id, self._updated_tags(name, id) )
            objects.append(to)
        walkers = self.world.get_actors().filter('walker*')
        for w in walkers:
            tag = 'Pedestrians'
            id = w.id
            name = w.type_id
            box = w.bounding_box
            trans = w.get_transform()
            to = TaggedObject(self.world, box, trans, id, self._updated_tags(tag, id) )
            objects.append(to)
        # # Some of the stop sign box data in map town03 are not exactly on the "stop" sign. 
        # # Not sure how to fix it. Ignore the stop sign
        # stops = self.world.get_actors().filter('traffic.stop')
        # for w in stops:
        #     tag = 'traffic.stop'
        #     id = w.id
        #     name = w.type_id
        #     box = w.bounding_box
        #     trans = w.get_transform()
        #     to = TaggedObject(self.world, box, trans, id, [tag])
        #     objects.append(to)
        return objects


# ==============================================================================
# -- ClientSideBoundingBoxes ---------------------------------------------------
# ==============================================================================
class ClientSideBoundingBoxes(object):
    """
    This is a module responsible for creating 3D bounding boxes and drawing them
    client-side on pygame surface.
    """

    @staticmethod
    def get_matrix(transform):
        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix    

    @staticmethod
    def get_list_transform(actors, sensor):
        t_list = []
        for vehicle in actors:
            v = vehicle.trans
            transform = [v.location.x , v.location.y , v.location.z , v.rotation.roll , v.rotation.pitch , v.rotation.yaw]
            t_list.append(transform)
        t_list = np.array(t_list).reshape((len(t_list),6))
        
        transform_h = np.concatenate((t_list[:,:3],np.ones((len(t_list),1))),axis=1)
        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        transform_s = np.dot(world_sensor_matrix, transform_h.T).T
        
        return t_list , transform_s

    @staticmethod
    def filter_distance(actors, v_transform, v_transform_s, sensor, max_dist=100):
        s = sensor.get_transform()
        s_transform = np.array([s.location.x , s.location.y , s.location.z])
        dist2 = np.sum(np.square(v_transform[:,:3] - s_transform), axis=1)
        selector = dist2 < (max_dist**2)
        vehicles_list_f = [v for v, s in zip(actors, selector) if s]
        v_transform_f = v_transform[selector,:]
        v_transform_s_f = v_transform_s[selector,:] 
        return vehicles_list_f , v_transform_f , v_transform_s_f

    ### Remove vehicles that are not in the FOV of the sensor
    @staticmethod
    def filter_angle(vehicles_list, v_transform, v_transform_s, sensor):
        attr_dict = sensor.attributes
        VIEW_FOV = float(attr_dict['fov'])
        v_angle = np.arctan2(v_transform_s[:,1],v_transform_s[:,0]) * 180 / np.pi

        selector = np.array(np.absolute(v_angle) < (int(VIEW_FOV)/2))
        #vehicles_list_f = [v for v, s in zip(vehicles_list, selector) if s]
        vehicles_list_f = []
        for v, s, angle in zip(vehicles_list, selector, v_angle):
            if s:
                face = (sensor.get_transform().rotation.yaw - v.trans.rotation.yaw + angle.getA1()[0]) % 360
                tag = v.tag['name']
                # if tag.find('light')>=0 or tag.find('speed_limit')>=0:
                #     print(f'actor:{v.actor_id} - {v.tag["name"]} - face: {face} ')
                # For Traffic Light and SpeedLimit, only label those objects on a special angle. Do not label it on back side
                if (tag.find('light')>=0 and face>240 and face <300) or \
                   (tag.find('speed_limit')>=0 and face>200 and face <340):
                    vn = f' - face - {face:.0f}'
                else:
                    vn = f' - back - {face:.0f}'
                    if tag.find('light')>=0 or tag.find('speed_limit')>=0:
                        continue
                vehicles_list_f.append(v)

        v_transform_f = v_transform[selector[:,0],:]
        v_transform_s_f = v_transform_s[selector[:,0],:]
        return vehicles_list_f , v_transform_f , v_transform_s_f

    @staticmethod
    def filter_angle_distance(actors, sensor, max_dist=100):
        vehicles_transform , vehicles_transform_s = ClientSideBoundingBoxes.get_list_transform(actors, sensor)
        actors , vehicles_transform , vehicles_transform_s = ClientSideBoundingBoxes.filter_distance(actors, vehicles_transform, vehicles_transform_s, sensor, max_dist)
        actors , vehicles_transform , vehicles_transform_s = ClientSideBoundingBoxes.filter_angle(actors, vehicles_transform, vehicles_transform_s, sensor)
        return actors

    @staticmethod
    def p3d_to_p2d_bb(p3d_bb):
        min_x = np.amin(p3d_bb[:,0])
        min_y = np.amin(p3d_bb[:,1])
        max_x = np.amax(p3d_bb[:,0])
        max_y = np.amax(p3d_bb[:,1])
        p2d_bb = np.array([[min_x,min_y] , [max_x,max_y]])
        return p2d_bb

    ### Apply occlusion filter based on resized bounding box depth values
    @staticmethod
    def filter_occlusion_bbox(bounding_boxes, actors, sensor, depth_meter, depth_margin=-1, patch_ratio=0.5, resize_ratio=0.5, debug = False):
        filtered_bboxes = []
        filtered_vehicles = []
        filtered_v_class = []
        filtered_out = {}
        removed_bboxes = []
        removed_vehicles = []
        removed_v_class = []
        removed_out = {}
        selector = []
        patches = []
        patch_delta = []
        _, v_transform_s = ClientSideBoundingBoxes.get_list_transform(actors, sensor)
        for v, vs, bbox in zip(actors,v_transform_s,bounding_boxes):
            dist = vs[:,0]
            extent = v.box.extent
            #depth_margin = (v.bounding_box.extent.x**2+v.bounding_box.extent.y**2)**0.5 + 0.25
            depth_margin = (extent.x**2+extent.y**2)**0.5  + 0.25
            uc = int((bbox[0,0]+bbox[1,0])/2)
            vc = int((bbox[0,1]+bbox[1,1])/2)
            wp = int((bbox[1,0]-bbox[0,0])*resize_ratio/2)
            hp = int((bbox[1,1]-bbox[0,1])*resize_ratio/2)
            u1 = uc-wp
            u2 = uc+wp
            v1 = vc-hp
            v2 = vc+hp
            depth_patch = np.array(depth_meter[v1:v2,u1:u2])
            dist_delta = dist-depth_patch
            dist2 = sensor.get_transform().location.distance(v.trans.location)
            s_patch = np.array(dist_delta < depth_margin)
            s = np.sum(s_patch) > s_patch.shape[0]*s_patch.shape[1]*patch_ratio
            if debug:
                print(f'{v.tag["name"]} - dist: {dist} - dist2:{dist2} - depth_margin:{depth_margin} -s_patch:{s_patch.shape} bbox:{bbox} {v.trans} {v.box} s:{s}\n{dist_delta}')
            selector.append(s)
            nbox = np.array([[u1,v1],[u2,v2]])
            patches.append((nbox,v))
            patch_delta.append(dist_delta)
        
        for bbox,v,s in zip(bounding_boxes,actors,selector):
            if s:
                filtered_bboxes.append((bbox,v))
                filtered_vehicles.append((bbox,v))
            else:
                removed_bboxes.append((bbox,v))
                removed_vehicles.append((bbox,v))
        filtered_out['bbox']=filtered_bboxes
        filtered_out['vehicles']=filtered_vehicles
        removed_out['bbox']=removed_bboxes
        removed_out['vehicles']=removed_vehicles
            
        return filtered_out, removed_out, patches

    @staticmethod
    def get_bounding_boxes(actors, camera, carla_rgb, depth_meter, debug):
        """
        Creates 3D bounding boxes based on carla vehicle list and camera.
        """

        vehicles = ClientSideBoundingBoxes.filter_angle_distance(actors , camera, max_distance)
        #print(f'Only {len(vehicles)} actors left after filter the angle and distance')
        bounding_boxes_3d = []
        for vehicle in vehicles:
            try:
                box = ClientSideBoundingBoxes.get_bounding_box(vehicle, camera)
                bounding_boxes_3d.append(box)
            except Exception as e:
                print(e)
                print(vehicle)
        bounding_boxes_2d = [ClientSideBoundingBoxes.p3d_to_p2d_bb(bbox) for bbox in bounding_boxes_3d]
        filtered_out,removed_out,depth_area = ClientSideBoundingBoxes.filter_occlusion_bbox(bounding_boxes_2d,vehicles,camera,depth_meter,depth_margin, patch_ratio, resize_ratio, debug)
        if debug:
            print(f'Only {len(filtered_out["bbox"])} actors left after filter {len(vehicles)} objects for the occlusion')
       
        return filtered_out['bbox'] , depth_area, removed_out['bbox']

    @staticmethod
    def draw_bounding_boxes(display, bounding_boxes, BB = False):
        """
        Draws bounding boxes on pygame display.
        """
        notes = ''
        speedsign = False
        bb_surface = pygame.Surface((VIEW_WIDTH, VIEW_HEIGHT))
        bb_surface.set_colorkey((0, 0, 0))
        if BB:
            DR_COLOR = BB_COLOR
        else:
            DR_COLOR = D_COLOR
        for p2d_points,v in bounding_boxes:
            point_0 = [p2d_points[0,0] , p2d_points[0,1]]
            point_1 = [p2d_points[1,0] , p2d_points[0,1]]
            point_2 = [p2d_points[1,0] , p2d_points[1,1]]
            point_3 = [p2d_points[0,0] , p2d_points[1,1]]
            # draw lines
            pygame.draw.line(bb_surface, DR_COLOR, point_0, point_1)
            pygame.draw.line(bb_surface, DR_COLOR, point_1, point_2)
            pygame.draw.line(bb_surface, DR_COLOR, point_2, point_3)
            pygame.draw.line(bb_surface, DR_COLOR, point_3, point_0)
            if BB:
                cx = (p2d_points[0,0] + p2d_points[1,0]) / 2.0 / float(VIEW_WIDTH)
                cy = (p2d_points[0,1] + p2d_points[1,1]) / 2.0 / float(VIEW_HEIGHT)
                w = (p2d_points[1,0] - p2d_points[0,0]) / float(VIEW_WIDTH)
                h = (p2d_points[1,1] - p2d_points[0,1]) / float(VIEW_HEIGHT)
                # Label different type of objects when larger than certain size
                tag = v.tag['name']
                if (h*float(VIEW_HEIGHT)>15 and tag.find('traffic.stop')<0) or \
                   (h*float(VIEW_HEIGHT)>13 and tag.find('speed_limit')>=0) or \
                   (h*float(VIEW_HEIGHT)>30 and tag.find('traffic.stop')>=0) or \
                   (h*float(VIEW_HEIGHT)>6 and tag.find('traffic.light')>=0):
                    # for special debug. only save data to disk for several type of objects ****
                    # if tag in ['traffic.speed_limit.30', 'traffic.speed_limit.60', 'traffic.speed_limit.90', 
                    #             'traffic.light.Red', 'traffic.light.Green', 'traffic.light.Yellow']:
                    #     speedsign = True
                    notes += f"{v.tag['index']} {cx:.5f} {cy:.5f} {w:.5f} {h:.5f}\n"
                    font2 = pygame.font.SysFont('didot.ttc', 24)
                    img2 = font2.render(f'{tag}', True, DR_COLOR)
                    bb_surface.blit(img2, point_0)
        display.blit(bb_surface, (0, 0))
        # for special debug ****
        # if not speedsign:
        #     notes = ''
        return notes
        

    @staticmethod
    def get_bounding_box(actor, camera):
        """
        Returns 3D bounding box for a vehicle based on camera view.
        """

        bb_cords = ClientSideBoundingBoxes._create_bb_points(actor)
        cords_x_y_z = ClientSideBoundingBoxes._vehicle_to_sensor(bb_cords, actor, camera)[:3, :]
        cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
        bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
        camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
        return camera_bbox

    @staticmethod
    def _create_bb_points(actor):
        """
        Returns 3D bounding box for a vehicle.
        """

        cords = np.zeros((8, 4))
        extent = actor.box.extent
        cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
        cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
        cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
        cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
        cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
        cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
        cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
        cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
        return cords

    @staticmethod
    def _vehicle_to_sensor(cords, actor, sensor):
        """
        Transforms coordinates of a vehicle bounding box to sensor.
        """

        world_cord = ClientSideBoundingBoxes._vehicle_to_world(cords, actor)
        sensor_cord = ClientSideBoundingBoxes._world_to_sensor(world_cord, sensor)
        return sensor_cord

    @staticmethod
    def _vehicle_to_world(cords, actor):
        """
        Transforms coordinates of a vehicle bounding box to world.
        """

        bb_transform = carla.Transform(actor.box.location)
        bb_vehicle_matrix = ClientSideBoundingBoxes.get_matrix(bb_transform)
        vehicle_world_matrix = ClientSideBoundingBoxes.get_matrix(actor.trans)
        bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
        world_cords = np.dot(bb_world_matrix, np.transpose(cords))
        return world_cords

    @staticmethod
    def _world_to_sensor(cords, sensor):
        """
        Transforms world coordinates to sensor.
        """

        sensor_world_matrix = ClientSideBoundingBoxes.get_matrix(sensor.get_transform())
        world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
        sensor_cords = np.dot(world_sensor_matrix, cords)
        return sensor_cords

    @staticmethod
    def get_matrix(transform):
        """
        Creates matrix from carla transform.
        """

        rotation = transform.rotation
        location = transform.location
        c_y = np.cos(np.radians(rotation.yaw))
        s_y = np.sin(np.radians(rotation.yaw))
        c_r = np.cos(np.radians(rotation.roll))
        s_r = np.sin(np.radians(rotation.roll))
        c_p = np.cos(np.radians(rotation.pitch))
        s_p = np.sin(np.radians(rotation.pitch))
        matrix = np.matrix(np.identity(4))
        matrix[0, 3] = location.x
        matrix[1, 3] = location.y
        matrix[2, 3] = location.z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        return matrix


# ==============================================================================
# -- BasicSynchronousClient ----------------------------------------------------
# ==============================================================================


class BasicSynchronousClient(object):
    """
    Basic implementation of a synchronous client.
    """

    def __init__(self):
        self.client = None
        self.world = None
        self.camera = None
        self.depth = None
        self.car = None
        self.npc_list = []
        self.npc_walker_list = []
        self.spawn_points = []

        self.display = None
        self.capture = True
        self.depth_dbg = False
        self.autopilot = False
        self.actors = []

    def camera_blueprint(self):
        """
        Returns camera blueprint.
        """

        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        camera_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        camera_bp.set_attribute('fov', str(VIEW_FOV))
        return camera_bp

    def depth_blueprint(self):
        """
        Returns depth blueprint.
        """

        depth_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', str(VIEW_WIDTH))
        depth_bp.set_attribute('image_size_y', str(VIEW_HEIGHT))
        depth_bp.set_attribute('fov', str(VIEW_FOV))
        return depth_bp

    def set_synchronous_mode(self, synchronous_mode):
        """
        Sets synchronous mode.
        """

        settings = self.world.get_settings()
        settings.synchronous_mode = synchronous_mode
        settings.fixed_delta_seconds = 0.1
        # settings.substepping = True
        # settings.max_substep_delta_time = 0.1
        # settings.max_substeps = 1
        self.world.apply_settings(settings)

    def setup_car(self):
        """
        Spawns actor-vehicle to be controled.
        """

        self.spawn_points = self.world.get_map().get_spawn_points()
        car_bp = self.world.get_blueprint_library().filter('vehicle.*')[0]
        try_time = 5
        while try_time>0:
            try:
                location = random.choice(self.spawn_points)
                self.car = self.world.spawn_actor(car_bp, location)
                break
            except Exception as e:
                try_time = try_time - 1
                if try_time==0:
                    raise(e)
        tm = self.client.get_trafficmanager()
        tm.ignore_lights_percentage(self.car,100)

    def setup_npc(self, number_of_npc=50):
        self.spawn_points = self.world.get_map().get_spawn_points()
        number_of_spawn_points = len(self.spawn_points)

        if number_of_npc < number_of_spawn_points:
            random.shuffle(self.spawn_points)
        elif number_of_npc > number_of_spawn_points:
            number_of_npc = number_of_spawn_points

        blueprints = self.world.get_blueprint_library().filter('vehicle.*')
        #blueprints = [b for b in blueprints if int(b.get_attribute('number_of_wheels'))==2]
        batch = []
        SetAutopilot = carla.command.SetAutopilot
        SetVehicleLightState = carla.command.SetVehicleLightState
        FutureActor = carla.command.FutureActor

        for n, transform in enumerate(self.spawn_points):
            if n >= number_of_npc:
                break
            blueprint = blueprints[n % len(blueprints)]
            batch.append(carla.command.SpawnActor(blueprint, transform)
            .then(SetAutopilot(FutureActor, True)))
            self.spawn_points.pop(0)

        for response in self.client.apply_batch_sync(batch):
            #print(type(response))
            #print(response)
            self.npc_list.append(response.actor_id)
                
    def setup_walker(self, number_of_walker=50):
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
        for n, spawn_point in enumerate(spawn_points):
            walker_bp = blueprintsWalkers[n % len(blueprintsWalkers)]
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
            batch.append(carla.command.SpawnActor(walker_bp, spawn_point))
        results = self.client.apply_batch_sync(batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                print(f"spawn walker {i} error:", results[i].error)
            else:
                self.npc_walker_list.append(results[i].actor_id)
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = self.world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(self.npc_walker_list)):
            batch.append(carla.command.SpawnActor(walker_controller_bp, carla.Transform(), self.npc_walker_list[i]))
        results = self.client.apply_batch_sync(batch, True)
        for i in range(len(results)):
            if results[i].error:
                print(f"mange walker {i} error:", results[i].error)
            else:
                self.npc_walker_list.append(results[i].actor_id)
        
        # 4. we put altogether the walkers and controllers id to get the objects from their id
        self.world.tick()
        all_actors = self.world.get_actors(self.npc_walker_list)

        self.world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(int((len(all_actors)+1)/2),len(all_actors)):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(self.world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[i-int((len(all_actors)+1)/2)]))
        self.world.tick()

    def setup_camera(self):
        """
        Spawns actor-camera to be used to render view.
        Sets calibration for client-side boxes rendering.
        """

        camera_transform = carla.Transform(carla.Location(x=1.5, z=1.5))
        
        self.camera = self.world.spawn_actor(self.camera_blueprint(), camera_transform, attach_to=self.car)
        #self.camera.listen(lambda image: weak_self().set_image(weak_self, image))
        self.camera.listen(camera_queue.put)

        self.depth = self.world.spawn_actor(self.depth_blueprint(), camera_transform, attach_to=self.car)
        self.depth.listen(depth_queue.put)

        calibration = np.identity(3)
        calibration[0, 2] = VIEW_WIDTH / 2.0
        calibration[1, 2] = VIEW_HEIGHT / 2.0
        calibration[0, 0] = calibration[1, 1] = VIEW_WIDTH / (2.0 * np.tan(VIEW_FOV * np.pi / 360.0))
        self.camera.calibration = calibration
        self.depth.calibration = calibration

    def control(self, car):
        """
        Applies control to main car based on pygame pressed keys.
        Will return True If ESCAPE is hit, otherwise False to end main loop.
        """

        keys = pygame.key.get_pressed()
        if keys[K_ESCAPE]:
            return True
        if keys[K_q]:
            self.depth_dbg = ~ self.depth_dbg
        if keys[K_p]:
            self.autopilot = not self.autopilot
            self.car.set_autopilot(self.autopilot)

        control = car.get_control()
        control.throttle = 0
        if keys[K_w]:
            control.throttle = 1
            control.reverse = False
        elif keys[K_s]:
            control.throttle = 1
            control.reverse = True
        if keys[K_a]:
            control.steer = max(-1., min(control.steer - 0.05, 0))
        elif keys[K_d]:
            control.steer = min(1., max(control.steer + 0.05, 0))
        else:
            control.steer = 0
        control.hand_brake = keys[K_SPACE]
        if self.depth_dbg>0:
            print('control:', control)
        car.apply_control(control)
        return False

    def render(self, display, camera_img):
        """
        Transforms image from camera sensor and blits it to main pygame display.
        """

        if camera_img is not None:
            array = np.frombuffer(camera_img.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (camera_img.height, camera_img.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            display.blit(surface, (0, 0))


    def init_traffic_light(self):
        traffics = self.world.get_actors()
        for t in traffics:
            if t.type_id == 'traffic.traffic_light':
                # print(f'Original red time: {t.get_red_time()}  yellow time: {t.get_red_time()} green time: {t.get_red_time()}')
                t.set_yellow_time(2.0)
                t.set_green_time(1.1)
                t.set_red_time(1.1)
                # print(f'After red time: {t.get_red_time()}  yellow time: {t.get_red_time()} green time: {t.get_red_time()}')
    def game_loop(self):
        """
        Main program loop.
        """

        try:
            pygame.init()

            self.client = carla.Client('127.0.0.1', 2000)
            self.client.set_timeout(12.0)
            #self.world = self.client.get_world()
            self.world = self.client.load_world(city)
            if weather_id>=0 and weather_id<len(weathers):
                self.world.set_weather(weathers[weather_id])
            self.setup_npc(npc_car_num)
            self.setup_walker(npc_walker_num)
            self.setup_car()
            self.setup_camera()

            self.display = pygame.display.set_mode((VIEW_WIDTH, VIEW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)

            self.set_synchronous_mode(True)
            self.init_traffic_light()
            tagged_objects = TaggedObjects(self.world)
            self.world.tick()
            oldnotes = ''
            global fileframe
            while True:
                get_thread_id()
                camera_queue.queue.clear()
                depth_queue.queue.clear()
                self.world.tick()
                #print('after a tick' , time.time())
                camera_img = camera_queue.get()
                depth_img = depth_queue.get()
                bgra_image = np.ndarray(shape=(depth_img.height, depth_img.width, 4), dtype=np.uint8, buffer=depth_img.raw_data)
                scales = np.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000
                depth_meter = np.dot(bgra_image, scales).astype(np.float32)

                actors = tagged_objects.get_all_objects()
                bounding_boxes, depth_patches, remove_out = ClientSideBoundingBoxes.get_bounding_boxes(actors, self.camera, camera_img, depth_meter, self.depth_dbg)
                self.capture = True
                self.render(self.display, camera_img)
                # Save image and label data to disk as yolov5 format
                fname = f'{city}_{starttime}_{fileframe:06}'
                fileframe += 1
                ClientSideBoundingBoxes.draw_bounding_boxes(self.display, remove_out)
                notes = ClientSideBoundingBoxes.draw_bounding_boxes(self.display, bounding_boxes, BB = True)
                if notes and notes!=oldnotes:
                    oldnotes = notes
                    if is_save_to_disk:
                        camera_img.save_to_disk(os.path.join(img_folder, fname+".png"))
                        pygame.image.save(self.display, os.path.join(chk_folder, fname+".png"))
                        with open(os.path.join(txt_folder, fname+".txt"), "w") as text_file:
                            text_file.write(notes)
                # indicate if now is autopilot model
                font2 = pygame.font.SysFont('didot.ttc', 24)
                img2 = font2.render(f'autopilot:{self.autopilot}', True, BB_COLOR)
                self.display.blit(img2, (0,0))
                pygame.display.flip()
                pygame.event.pump()
                if self.control(self.car):
                    return
        except Exception as e:
            print(e)
            traceback.print_exc() 
        finally:
            self.set_synchronous_mode(False)
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.world.get_actors().filter('vehicle.*')])
            self.camera.destroy()
            self.car.destroy()
            self.depth.destroy()
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.npc_list])
            self.client.apply_batch([carla.command.DestroyActor(x) for x in self.npc_walker_list])
            pygame.quit()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    Initializes the client-side bounding box demo.
    """

    try:
        client = BasicSynchronousClient()
        client.game_loop()
    finally:
        print('EXIT')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--view-width', type=int, default=800, help='initial view width')
    parser.add_argument('--view-height', type=int, default=560, help='initial view height')
    parser.add_argument('--view-max-distance', type=int, default=40, help='label objects in this distance')
    parser.add_argument('--npc-car-number', type=int, default=71, help='spawn car number')
    parser.add_argument('--npc-walker-number', type=int, default=21, help='spawn walker number')
    parser.add_argument('--map-name', type=str, default='Town02', help='carla map name')
    parser.add_argument('--map-weather', type=int, default=-1, help='carla map name')
    parser.add_argument('--img-saved-dir', type=str, default='./data/img', help='save camera image to this directory')
    parser.add_argument('--label-saved-dir', type=str, default='./data/txt', help='save label file to this directory')
    parser.add_argument('--check-saved-dir', type=str, default='./data/chk', help='save labeled image file to this directory for debug')
    parser.add_argument('--save-to-disk', type=bool, default=False, help='save labeled image file to disk')

    opt = parser.parse_args()
    VIEW_WIDTH = opt.view_width
    VIEW_HEIGHT = opt.view_height
    max_distance = opt.view_max_distance
    npc_car_num = opt.npc_car_number
    npc_walker_num = opt.npc_walker_number
    city = opt.map_name
    img_folder = opt.img_saved_dir
    txt_folder = opt.label_saved_dir
    chk_folder = opt.check_saved_dir
    is_save_to_disk = opt.save_to_disk
    weather_id = opt.map_weather
    main()
