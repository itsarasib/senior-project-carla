import carla
import time 
def main():
    try:
        # Connect the client and set up bp library and spawn points
        client = carla.Client('localhost', 2000)
        client.set_timeout(120.0)
        world = client.get_world()
        bp_lib = world.get_blueprint_library()  

        spawn_points = world.get_map().get_spawn_points()
        spawn_pt = carla.Transform(carla.Location(spawn_points[219].location), carla.Rotation(pitch=0, yaw=270, roll=0))  

        # Add the ego vehicled
        vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
        vehicle = world.try_spawn_actor(vehicle_bp, spawn_pt)
        # Wait for the user to press Enter to exite
        input("Press Enter to exit and remove the vehicle...")
    finally:
       if vehicle.is_alive:
            vehicle.destroy()
            print("Vehicle destroyed.")  
if __name__ == '__main__':
    main()

