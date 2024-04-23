#all imports
import carla #the sim library itself
import time # to set a delay after each photo
import cv2 #to work with images from cameras
import numpy as np #in this example to change image representation - re-shaping

# connect to the sim 
client = carla.Client('localhost', 2000)

#define environment/world and get possible places to spawn a car
world = client.get_world()
spawn_points = world.get_map().get_spawn_points()
#look for a blueprint of Mini car
vehicle_bp = world.get_blueprint_library().filter('*mini*')

start_point = spawn_points[0]
vehicle = world.try_spawn_actor(vehicle_bp[0], start_point)
#setting RGB Camera - this follow the approach explained in a Carla video
# link: https://www.youtube.com/watch?v=om8klsBj4rc&t=1184s

#camera mount offset on the car - you can tweak these to have the car in view or not
CAMERA_POS_Z = 3 
CAMERA_POS_X = -5 

camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '640') # this ratio works in CARLA 9.14 on Windows
camera_bp.set_attribute('image_size_y', '360')

camera_init_trans = carla.Transform(carla.Location(z=CAMERA_POS_Z,x=CAMERA_POS_X))
#this creates the camera in the sim
camera = world.spawn_actor(camera_bp,camera_init_trans,attach_to=vehicle)

def camera_callback(image,data_dict):
    data_dict['image'] = np.reshape(np.copy(image.raw_data),(image.height,image.width,4))

image_w = camera_bp.get_attribute('image_size_x').as_int()
image_h = camera_bp.get_attribute('image_size_y').as_int()

camera_data = {'image': np.zeros((image_h,image_w,4))}
# this actually opens a live stream from the camera
camera.listen(lambda image: camera_callback(image,camera_data))

# route planning bit
import sys
sys.path.append('C:\CARLA_0.9.15\WindowsNoEditor\PythonAPI\carla') # tweak to where you put carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

point_a = start_point.location #we start at where the car is

sampling_resolution = 1
grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)

# now let' pick the longest possible route
distance = 0
for loc in spawn_points: # we start trying all spawn points 
                            #but we just exclude first at zero index
    cur_route = grp.trace_route(point_a, loc.location)
    if len(cur_route)>distance:
        distance = len(cur_route)
        route = cur_route
#draw the route in sim window - Note it does not get into the camera of the car
for waypoint in route:
    world.debug.draw_string(waypoint[0].transform.location, '^', draw_shadow=False,
        color=carla.Color(r=0, g=0, b=255), life_time=60.0,
        persistent_lines=True)
    
# the cheating loop of moving the car along the route
for waypoint in route:
    
    # move the car to current waypoint
    vehicle.set_transform(waypoint[0].transform)
    # Dispaly with imshow
    cv2.imshow('Fake self-driving',camera_data['image'])
    cv2.waitKey(50)
    
time.sleep(2)
cv2.destroyAllWindows()
camera.stop() # this is the opposite of camera.listen
for actor in world.get_actors().filter('*vehicle*'):
    actor.destroy()
for sensor in world.get_actors().filter('*sensor*'):
    sensor.destroy()