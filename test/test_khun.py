import carla
import math 
import random 
import time 
import numpy as np


#Connect the client and set up bp library and spawn points
client = carla.Client('localhost', 2000)
client.set_timeout(120.0)
world = client.get_world()
bp_lib = world.get_blueprint_library()

spawn_points = carla.Transform(carla.Location(x=248, y=37.5, z=0.20000), carla.Rotation(pitch=0, yaw=0, roll=0))

#Add the ego vehicle
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020') 
vehicle = world.try_spawn_actor(vehicle_bp, spawn_points)