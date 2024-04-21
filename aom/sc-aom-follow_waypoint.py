# import carla
# import time
# import random
# def main():
#     try:
#         # Connect to the CARLA server
#         client = carla.Client('localhost', 2000)
#         client.set_timeout(10.0)
#         # Get the world
#         world = client.get_world()
#         # Get the blueprint for the vehicle
#         blueprint_library = world.get_blueprint_library()
#         vehicle = blueprint_library.find('vehicle.volkswagen.t2')
#         traffic_manager = client.get_trafficmanager()
#         traffic_manager.ignore_lights_percentage(vehicle, random.randint(0,50))
#         if not vehicle:
#             print("Vehicle blueprint not found.")
#             return
#         # Get all possible spawn points
#         spawn_points = world.get_map().get_spawn_points()
#         for i, spawn_point in enumerate(spawn_points):
#             world.debug.draw_string(spawn_point.location, str(i), life_time=10)
#         spawn_point  = spawn_points[219]
#         route_1_indices = [100,267,188,1,251,184]
#         route_1 = []
#         for ind in route_1_indices:
#             route_1.append(spawn_points[ind].location)

#         world.debug.draw_string(spawn_point.location, 'Spawn point 1', life_time=30, color=carla.Color(255,0,0))
        
#         for ind in route_1_indices:
#             spawn_points[ind].location
#             world.debug.draw_string(spawn_points[ind].location, str(ind), life_time=60, color=carla.Color(255,0,0))
        
#         # Set delay to create gap between spawn times
#         spawn_delay = 20
#         counter = spawn_delay
#         # Alternate between spawn points
#         alt = Falsea

#         while True:
#             world.tick()
          
#             # Spawn vehicle only after delay
#             if counter == spawn_delay:
#                 # Alternate spawn points
#                 if alt:
#                     vehicle = world.try_spawn_actor(vehicle, spawn_point)
#                     vehicle.set_autopilot(True) # Give TM control over vehicle

#                     # Set parameters of TM vehicle control, we don't want lane changes
#                     traffic_manager.update_vehicle_lights(vehicle, True)
#                     traffic_manager.random_left_lanechange_percentage(vehicle, 0)
#                     traffic_manager.random_right_lanechange_percentage(vehicle, 0)
#                     traffic_manager.auto_lane_change(vehicle, False)

                
#             # Keep the script alive until you decide to exit
#             input("Press Enter to exit and remove the vehicle...")
#     finally:
#                 # Ensure the vehicle is destroyed to free resources
#                 if 'vehicle' in locals() and vehicle.is_alive:
#                     vehicle.destroy()
#                     print("Vehicle destroyed.")

import carla
import time
import random

def main():
    # Connect to the CARLA server
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)  # seconds
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.audi.a2')

    # Fetch the spawn points from the map
    spawn_points = world.get_map().get_spawn_points()

    # Define the initial spawn point and spawn the vehicle
    spawn_point =carla.Transform(carla.Location(spawn_points[219].location), carla.Rotation(pitch=0, yaw=90, roll=0))  
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    destination = carla.Location(carla.Location(spawn_points[251].location))
    # Enable autopilot using Traffic Manager
    tm = client.get_trafficmanager()
    vehicle.set_autopilot(True, tm.get_port())
    tm.ignore_lights_percentage(vehicle, random.randint(0,100))

    # Define the route using the indices of additional spawn points
    route_indices = [215, 52, 258, 298,88,55,184,251]
    route_locations = [spawn_points[i].location for i in route_indices]  # Extracting locations directly
    # Set the route for the vehicle using the traffic manager
    tm.set_path(vehicle, route_locations)  # Use locations instead of waypoints
    
    # Run the simulation for some time to observe the vehicle
    try:
        while True:
            vehicle_location = vehicle.get_location()
            if vehicle_location.distance(destination) < 2.0:  # Using a distance threshold :
                print("Vehicle has reached the destination.")
                vehicle.set_autopilot(False)  # Disable autopilot
                vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0))  # Apply brake
                break 
            time.sleep(1)
        
        input("Press Enter to exit and remove the vehicle...")
    finally:
        if vehicle.is_alive:
            vehicle.destroy()
            print("Vehicle destroyed.") 

if __name__ == '__main__':
    main()

