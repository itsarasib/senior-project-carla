# import carla
# import time

# def main():
#     try:
#         # Connect to the CARLA server
#         client = carla.Client('localhost', 2000)
#         client.set_timeout(10.0)

#         # Get the world
#         world = client.get_world()
        
#         # Get the map
#         map = world.get_map()

#         # Get the blueprint for the vehicle
#         blueprint_library = world.get_blueprint_library()
#         vehicle_bp = blueprint_library.find('vehicle.volkswagen.t2')
#         if not vehicle_bp:
#             print("Vehicle blueprint not found.")
#             return

#         # Get all possible spawn points
#         spawn_points = map.get_spawn_points()

#         if len(spawn_points) > max(190, 300):
#             start_spawn_point = spawn_points[190]
#             end_spawn_point = spawn_points[300]

#             # Spawn the vehicle at the start point
#             vehicle = world.spawn_actor(vehicle_bp, start_spawn_point)
#             print(f"Spawned {vehicle.type_id} at {start_spawn_point.location}")

#             # Set the vehicle to drive automatically towards the destination
#             vehicle.set_autopilot(True)
            
#             # Calculate the route
#             start_waypoint = map.get_waypoint(start_spawn_point.location)
#             end_waypoint = map.get_waypoint(end_spawn_point.location)
#             route = map.get_shortest_path(start_waypoint, end_waypoint)

#             # Drive the vehicle along the route
#             for waypoint in route:
#                 vehicle.set_transform(waypoint.transform)
#                 time.sleep(0.5)  # wait for a short time before moving to the next waypoint

#                 # Check if vehicle is close to the destination
#                 if vehicle.get_location().distance(end_spawn_point.location) < 2.0:
#                     print("Destination reached.")
#                     break

#         else:
#             print("Spawn point indices are out of range.")
#             return

#         # Keep the script alive until you decide to exit
#         input("Press Enter to exit and remove the vehicle...")

#     finally:
#         # Ensure the vehicle is destroyed to free resources
#         if 'vehicle' in locals() and vehicle.is_alive:
#             vehicle.destroy()
#             print("Vehicle destroyed.")

# if __name__ == '__main__':
#     main()

import carla
import time

def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        map = world.get_map()

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.volkswagen.t2')

        spawn_points = map.get_spawn_points()
        
        
        if len(spawn_points) > max(190, 300):
            start_spawn_point = spawn_points[190]
            end_spawn_point = spawn_points[300]

            vehicle = world.spawn_actor(vehicle_bp, start_spawn_point)
            print(f"Spawned {vehicle.type_id} at {start_spawn_point.location}")

            start_waypoint = map.get_waypoint(start_spawn_point.location)
            end_waypoint = map.get_waypoint(end_spawn_point.location)

            # Check if waypoints are valid
            if start_waypoint is None or end_waypoint is None:
                print("Failed to find valid waypoints.")
                return

            # Navigate to each waypoint until reaching the destination
            current_waypoint = start_waypoint
            while current_waypoint is not None:
                vehicle.set_transform(current_waypoint.transform)
                time.sleep(0.5)  # Simulate time for moving to the next waypoint

                # Check if the vehicle is close enough to the end waypoint
                if vehicle.get_location().distance(end_waypoint.transform.location) < 2.0:
                    print("Destination reached.")
                    break

                # Move to the next waypoint
                current_waypoint = current_waypoint.next(2.0)[0] if current_waypoint.next(2.0) else None

            input("Press Enter to exit and remove the vehicle...")

    finally:
        if 'vehicle' in locals() and vehicle.is_alive:
            vehicle.destroy()
            print("Vehicle destroyed.")

if __name__ == '__main__':
    main()
