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
    spawn_point = carla.Transform(carla.Location(spawn_points[31].location), carla.Rotation(pitch=0, yaw=270, roll=0))
    vehicle = world.spawn_actor(vehicle_bp, spawn_point)
    destination = carla.Location(spawn_points[199].location)

    tm = client.get_trafficmanager()
    vehicle.set_autopilot(True, tm.get_port())
    tm.ignore_lights_percentage(vehicle, 100)  # Ignore all traffic lights

    #  [31(lane2), 117(trigger, lane 2), 278(lane 3), 199(lane 3)]
    route_indices = [31, 117, 278, 199]
    route_locations = [spawn_points[i].location for i in route_indices]

    tm.set_path(vehicle, route_locations)

    lane_change_point = spawn_points[117].location  # Specify where you want the lane change
    # Example index where we want the lane change
    lane_changed = False  # Flag to check if lane change has occurred
    last_speed = 0
    
    try:
            while True:
                vehicle_location = vehicle.get_location()
                current_speed = vehicle.get_velocity().length()  # m/s

                if vehicle_location.distance(destination) < 5.0:
                    print("Vehicle has reached the destination.")
                    vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0))
                    break
                desired_speed = calculate_desired_speed(vehicle_location, route_locations, vehicle)
                # Manually adjust speed only when necessary
                if abs(current_speed - last_speed) >= 0.5:
                    adjust_speed(vehicle, current_speed, desired_speed)
                    last_speed = current_speed

                # Force a lane change at a specific waypoint
                if not lane_changed and vehicle_location.distance(lane_change_point) < 10.0:
                    tm.force_lane_change(vehicle, True)  # True for right lane change
                    lane_changed = True

                time.sleep(0.05)
            input("Press Enter to exit and remove the vehicle...")
    finally:  
        if vehicle.is_alive:
            vehicle.destroy()
            print("Vehicle destroyed.")
def adjust_speed(vehicle, current_speed, desired_speed):
    if current_speed < desired_speed:
        throttle = min(1.0, 0.5 + 0.1 * (desired_speed - current_speed))
        brake = 0.0
    else:
        throttle = 0.0
        brake = min(1.0, 0.1 * (current_speed - desired_speed))
    vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake))

def calculate_desired_speed(location, waypoints, vehicle):
    for i in range(len(waypoints) - 1):
        if location.distance(waypoints[i]) < 10:
            segment = waypoints[i+1] - waypoints[i]
            if abs(segment.x) > abs(segment.y):
                return 15.0  # Speed for straight paths
            else:
                return 10.0  # Reduced speed for curves
    return 15.0  # Default speed if not close to any waypoint

if __name__ == '__main__':
    main()