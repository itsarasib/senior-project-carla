import carla

def main():
    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Get the world
        world = client.get_world()

        # Get the blueprint for the vehicle
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.volkswagen.t2')
        if not vehicle_bp:
            print("Vehicle blueprint not found.")
            return

        # Get all possible spawn points
        spawn_points = world.get_map().get_spawn_points()

        # Ensure the spawn point index is within the list range
        if len(spawn_points) > 190:
            # Spawn at a specific location (index=190) with customized orientation
            start_spawn_point = carla.Transform(carla.Location(spawn_points[190].location), carla.Rotation(pitch=0, yaw=180, roll=0))
            # Spawn the vehicle
            vehicle = world.spawn_actor(vehicle_bp, start_spawn_point)
            print(f"Spawned {vehicle.type_id} at {start_spawn_point.location}")
        else:
            print("Specified spawn point index is out of range.")
            return

        # Keep the script alive long enough to see the vehicle
        input("Press Enter to exit and remove the vehicle...")

    finally:
        # Ensure the vehicle is destroyed to free resources
        if 'vehicle' in locals() and vehicle.is_alive:
            vehicle.destroy()
            print("Vehicle destroyed.")

if __name__ == '__main__':
    main()
