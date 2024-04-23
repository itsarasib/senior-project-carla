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

    lane_change_point = spawn_points[279].location  # Specify where you want the lane change
    lane_changed = False  # Flag to check if lane change has occurred

    try:
        
        while True:
            vehicle_location = vehicle.get_location()
            current_speed = vehicle.get_velocity().length() # m/s
            #desired_speed = calculate_desired_speed(vehicle_location, route_locations)
            
            # if abs(current_speed - last_speed) >= 1.0:
            #     print(f"Current Speed: {current_speed:.2f} m/s")
            #     last_speed = current_speed

            if vehicle_location.distance(destination) < 5.0:
                print("Vehicle has reached the destination.")
                vehicle.set_autopilot(False)
                vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0))
                break

            # Control logic for speed
            # if current_speed < desired_speed:
            #     throttle = min(1.0, 0.5 + 0.1 * (desired_speed - current_speed)) # Accelerate
            #     brake = 0.0 # No braking
            # else:
            #     throttle = 0.0 # No acceleration
            #     brake = min(1.0, 0.1 * (current_speed - desired_speed)) # Apply brakes
            #     return throttle, brake
            # vehicle.apply_control(carla.VehicleControl(throttle=throttle, brake=brake))

            # Check for lane change execution
            if not lane_changed and vehicle_location.distance(lane_change_point) < 5.0:
                vehicle.set_autopilot(False)
                vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.1))  # Adjust steer value for desired lane change
                time.sleep(1)  # Keep this control for a short time
                vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0))  # Straighten the vehicle after lane change
                time.sleep(1)
                vehicle.set_autopilot(True, tm.get_port())  # Re-enable autopilot after lane change
                lane_changed = True

            time.sleep(0.05)  # Control loop rate
        input("Press Enter to exit and remove the vehicle...")
    finally:
        
        if vehicle.is_alive:
            #input("Press Enter to exit and remove the vehicle...")
            vehicle.destroy()
            print("Vehicle destroyed.")

# def speed_control(current_speed, desired_speed):
#     if current_speed < desired_speed:
#         throttle = min(1.0, 0.5 + 0.1 * (desired_speed - current_speed))
#         brake = 0.0
#     else:
#         throttle = 0.0
#         brake = min(1.0, 0.1 * (current_speed - desired_speed))
#     return throttle, brake

def calculate_desired_speed(location, waypoints):
    for i in range(len(waypoints)-1):
        if location.distance(waypoints[i]) < 10:
            next_segment = waypoints[i+1] - waypoints[i]
            if abs(next_segment.x) > abs(next_segment.y):
                return 15.0  # Speed for straight paths
            else:
                return 10.0  # Reduced speed for curves
    return 15.0  # Default speed

# def execute_lane_change(vehicle):
#     vehicle.set_autopilot(False)
#     vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.1))  # Adjust steer value for desired lane change
#     time.sleep(1)  # Keep this control for a short time
#     vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0))  # Straighten the vehicle after lane change
#     time.sleep(1)
#     # tm = carla.Client('localhost', 2000).get_trafficmanager()
#     vehicle.set_autopilot(True, tm.get_port())  # Re-enable autopilot after lane change
#     lane_changed = True



if __name__ == '__main__':
    main()

#     try:
#         while True:
#             vehicle_location = vehicle.get_location()
#             if vehicle_location.distance(destination) < 2.0:
#                 print("Vehicle has reached the destination.")
#                 vehicle.set_autopilot(False)
#                 vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0))
#                 break

#             # Check if the vehicle is near the lane change point
#             if not lane_changed and vehicle_location.distance(lane_change_point) < 5.0:
#                 # Temporarily disable autopilot to change lanes manually
#                 vehicle.set_autopilot(False)
#                 vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.1))  # Adjust steer value for desired lane change
#                 time.sleep(1)  # Keep this control for a short time
#                 vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0))  # Straighten the vehicle after lane change
#                 time.sleep(0.5)
#                  # Stop the vehicle for 5-6 seconds
#                 vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0))
#                 time.sleep(5)  # Adjust duration for stop here
#                 vehicle.set_autopilot(True, tm.get_port())  # Re-enable autopilot after lane change
#                 lane_changed = True

#             time.sleep(1)

#         input("Press Enter to exit and remove the vehicle...")
#     finally:
#         if vehicle.is_alive:
#             vehicle.destroy()
#             print("Vehicle destroyed.")

# if __name__ == '__main__':
#     main()