#all imports
import carla #the sim library itself
import time # to set a delay after each photo
#import cv2 #to work with images from cameras
import numpy as np #in this example to change image representation - re-shaping
import math
import sys
import random
sys.path.append('C:\CARLA_0.9.15\WindowsNoEditor\PythonAPI\carla') # tweak to where you put carla
from agents.navigation.global_route_planner import GlobalRoutePlanner

# connect to the sim 
client = carla.Client('localhost', 2000)

world = client.get_world()

spectator = world.get_spectator()

# get map look at the map
town_map = world.get_map()
roads = town_map.get_topology()

# set up route generations
sampling_resolution = 1
grp = GlobalRoutePlanner(town_map, sampling_resolution)

# show first 10 road sections animated in a loop in an example


# make unique waypoints and show on the map 
# accumulate all waypoints all roads in a loop while making them unique on location x an y
unique_waypoints = []
for road in roads:
    
    cur_route = grp.trace_route(road[0].transform.location,road[1].transform.location)
    for wp in cur_route:
        if len(unique_waypoints)==0:
            unique_waypoints.append(wp[0]) #first waypoint is added regardless to start the list
        else:
            found = False
            for uwp in unique_waypoints: #check for same located waypoints and ignore if found
                if abs(uwp.transform.location.x - wp[0].transform.location.x) < 0.1 \
                            and abs(uwp.transform.location.y - wp[0].transform.location.y)<0.1 \
                            and abs(uwp.transform.rotation.yaw - wp[0].transform.rotation.yaw)<20:
                    found = True
            if not found:
                unique_waypoints.append(wp[0])

#let's show them on the map
for uwp in unique_waypoints:
    world.debug.draw_string(uwp.transform.location, '^', draw_shadow=False,
        color=carla.Color(r=0, g=0, b=255), life_time=60.0,
        persistent_lines=True)

#move spectator for top down view to see all points 
spectator_pos = carla.Transform(carla.Location(x=0,y=30,z=200),
                                carla.Rotation(pitch = -90, yaw = -90))
spectator.set_transform(spectator_pos)