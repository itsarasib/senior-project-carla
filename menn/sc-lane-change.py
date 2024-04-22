import carla
import time
import random

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    vehicle_bp = blueprint_library.find('vehicle.volkswagen.t2')

    spawn_points = world.get_map().get_spawn_points()

    spawn_point = carla.Transform(carla.Location(spawn_points[190].location), carla.Rotation(pitch=0, yaw=180, roll=0))
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    destination = carla.Location(spawn_points[300].location)

    tm = client.get_trafficmanager()
    vehicle.set_autopilot(True, tm.get_port())
    tm.ignore_lights_percentage(vehicle, 100) # Ignore all traffic lights random.randint(0,100)

    route_indices = [190, 63, 182, 5, 83, 300]
    route_locations = [spawn_points[i].location for i in route_indices]

    tm.set_path(vehicle, route_locations)

    lane_change_point = spawn_points[248].location  # Specify where you want the lane change
    lane_changed = False  # Flag to check if lane change has occurred

    try:
        while True:
            vehicle_location = vehicle.get_location()
            if vehicle_location.distance(destination) < 2.0:
                print("Vehicle has reached the destination.")
                vehicle.set_autopilot(False)
                vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0))
                break

            # Check if the vehicle is near the lane change point
            if not lane_changed and vehicle_location.distance(lane_change_point) < 5.0:
                # Temporarily disable autopilot to change lanes manually
                vehicle.set_autopilot(False)
                vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.1))  # Adjust steer value for desired lane change
                time.sleep(1)  # Keep this control for a short time
                vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0))  # Straighten the vehicle after lane change
                time.sleep(1)
                vehicle.set_autopilot(True, tm.get_port())  # Re-enable autopilot after lane change
                lane_changed = True

            time.sleep(1)

        input("Press Enter to exit and remove the vehicle...")
    finally:
        if vehicle.is_alive:
            vehicle.destroy()
            print("Vehicle destroyed.")

if __name__ == '__main__':
    main()