import time
import carla
import math

import rospy
from std_msgs.msg._String import String

rospy.init_node('Car_Info')
pub = rospy.Publisher("ch_carla", String, queue_size=10)

client = carla.Client('localhost',2000)
client.set_timeout(100.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

#model

SUV = client.get_world().get_blueprint_library().filter('etron')[0]
sedan = client.get_world().get_blueprint_library().filter('model3')[0]
pick_up = client.get_world().get_blueprint_library().filter('cybertruck')[0]
truck = client.get_world().get_blueprint_library().filter('carlacola')[0]
motorcycle = client.get_world().get_blueprint_library().filter('ninja')[0]
bicycle = client.get_world().get_blueprint_library().filter('crossbike')[0]

spawn_point1 = carla.Transform(carla.Location(x=106.251480, y=52.317177, z=0.20000), carla.Rotation(pitch=0.000000, yaw=-0.053956, roll=0.000000))
spawn_point2 = carla.Transform(carla.Location(x=106.248184, y=48.817181, z=0.200000), carla.Rotation(pitch=0.000000, yaw=-0.053956, roll=0.000000))

velocity1 = 50*5/18
velocity2 = 50*5/18
car_length = {SUV: 4.908 , sedan: 4.894 , pick_up: 5.381 , truck: 6.599 , motorcycle : 1.93, bicycle : 1.6}

blueprint = {"SUV":SUV, "sedan":sedan, "pick up" : pick_up, "truck":truck, "motorcycle" :motorcycle, "bicycle": bicycle}
car_ID = {"SUV":"car", "sedan":"car", "pick up" : "truck", "truck":"truck", "motorcycle" :"bike", "bicycle": "bike"}

###lane1
vehicle_actor1 = []
vehicle_setvelo1 = []
###lane2
vehicle_actor2 = []
vehicle_setvelo2 = []

def Select_Case():
    global velocity1,velocity2,spawn_point1,spawn_point2,car_lane1,car_lane2,car_gab1,car_gab2,timeout
    timeout = 30
    c = int(input("Case: "))
    if c == 1: ## A car per lane
        velocity1 -= 15*5/18
        velocity2 -= 15*5/18
        spawn_point1.location.x -= 75
        spawn_point2.location.x -= 75
        car_lane1 = ["SUV"]
        car_lane2 = ["sedan"]
        car_gab1 = [5,40]
        car_gab2 = [20,10]
        timeout += 0
    elif c == 2:## More than 1 car per lane
        velocity1 -= 15*5/18
        velocity2 -= 15*5/18
        spawn_point1.location.x -= 40
        spawn_point2.location.x -= 40
        car_lane1 = ["sedan","motorcycle","SUV"]
        car_lane2 = ["pick up","truck"]
        car_gab1 = [5,10]
        car_gab2 = [20]
        timeout += 0
    elif c == 3: ## spawn 7 car in lane1
        velocity1 -= 15*5/18
        velocity2 -= 0*5/18
        spawn_point1.location.x -= 15
        spawn_point2.location.x -= 15
        car = "SUV"
        gab = 10
        car_lane1 = []
        car_gab1 = []
        car_lane2 = []
        car_gab2 = []
        timeout += 0
        for i in range(7):
            car_lane1.append(car)
            car_gab1.append(gab)
    elif c == 4: ## Cars with different velocity
        velocity1 -= 5*5/18
        velocity2 -= 15*5/18
        spawn_point1.location.x -= 100
        spawn_point2.location.x -= 55
        car_lane1 = ["SUV"]
        car_lane2 = ["sedan"]
        car_gab1 = [5]
        car_gab2 = [20]
        timeout += 0
    elif c == 5: ## T2 into Car gap
        velocity1 -= 35*5/18
        velocity2 -= 35*5/18
        spawn_point1.location.x += 60
        spawn_point2.location.x -= 0
        car_lane1 = ["SUV","sedan"]
        car_lane2 = []
        car_gab1 = [35]
        car_gab2 = [20]
        timeout += 20

    for car in car_lane1:
        vehicle_actor1.append(blueprint[car])

    for car in car_lane2:
        vehicle_actor2.append(blueprint[car])

    ### draw conflict area
    t = timeout
    waypoints = client.get_world().get_map().generate_waypoints(distance=1)

    filtered_waypoints_16 = []
    filtered_waypoints_25 = []
    filtered_waypoints_735 = []

    for waypoint in waypoints:
        if(waypoint.road_id == 16):
            filtered_waypoints_16.append(waypoint)
        if(waypoint.road_id == 25):
            filtered_waypoints_25.append(waypoint)
        if(waypoint.road_id == 735):
            filtered_waypoints_735.append(waypoint)

    x = max(velocity1,velocity2)
    before_merge = round(x * 2.3*1.2 ) + 1
    after_merge = round(x * 2.3*1.2/2 ) + 1
    #merge_point at 64

    point_in16 = len(filtered_waypoints_16)
    point_in25 =len(filtered_waypoints_25)
    point_in735 =len(filtered_waypoints_735)

    all_point = []

    ## merge_point at point 64

    for i in range(68+after_merge*4):
        target_waypoint = filtered_waypoints_735[i]
        client.get_world().debug.draw_string(target_waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=255, b=0), life_time=t, persistent_lines=True)
        all_point.append(filtered_waypoints_735[i])

    before_merge -= 17
    if before_merge < 19:
        for i in range(before_merge*4):
            target_waypoint = filtered_waypoints_25[(point_in25-1) -i ]
            client.get_world().debug.draw_string(target_waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=255, b=0), life_time=t, persistent_lines=True)
            all_point.append(filtered_waypoints_25[i])
    else:
        for target_waypoint in filtered_waypoints_25:
            client.get_world().debug.draw_string(target_waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=255, b=0), life_time=t, persistent_lines=True)
            all_point.append(filtered_waypoints_25)

        before_merge -=19
        for i in range(before_merge*4):
            target_waypoint = filtered_waypoints_16[-(i+1)]
            client.get_world().debug.draw_string(target_waypoint.transform.location, 'O', draw_shadow=False, color=carla.Color(r=255, g=255, b=0), life_time=t, persistent_lines=True)
            all_point.append(filtered_waypoints_16[i])

#### control car

def spawn():
    ### lane1
    for i in range(int(len(vehicle_actor1))):
        vehicle = client.get_world().spawn_actor(vehicle_actor1[i], spawn_point1)
        vehicle_setvelo1.append(vehicle)
        if i < len(car_lane1) - 1:
            spawn_point1.location.x -= car_gab1[i]
            spawn_point1.location.x -=int(car_length[vehicle_actor1[i]])/2

    ### lane 2
    for a in range(int(len(vehicle_actor2))):
        vehicle = client.get_world().spawn_actor(vehicle_actor2[a], spawn_point2)
        vehicle_setvelo2.append(vehicle)
        if a < len(car_lane2) - 1:
            spawn_point2.location.x -= car_gab2[a]
            spawn_point2.location.x -= int(car_length[vehicle_actor2[a]])/2

def set_velocity():
    ###lane1
    for n in range(int(len(vehicle_setvelo1))):
        vehicle_setvelo1[n].enable_constant_velocity(carla.Vector3D(x=velocity1, y = 0.00, z = -0.50))

    ###lane2
    for b in range(int(len(vehicle_setvelo2))):
        vehicle_setvelo2[b].enable_constant_velocity(carla.Vector3D(x=velocity2 , y = 0.00, z = -0.50))

def destroy():
    ## destroy all car
    global vehicle_setvelo1, vehicle_setvelo2
    for carl1 in vehicle_setvelo1:
        carl1.destroy()
    for carl2 in vehicle_setvelo2:
        carl2.destroy()
    vehicle_setvelo1=[]
    vehicle_setvelo2=[]
result = []
output_sensor = []

def get_position():
    merge_position_x = 234.2
    for i in range(len(vehicle_setvelo1)) :
        data = []
        car_position1 = vehicle_setvelo1[i].get_location().x
        data.append(vehicle_setvelo1[i].id)
        data.append(car_ID[car_lane1[i]])
        car_distance1 = merge_position_x - car_position1 - car_length[vehicle_actor1[i]]/2
        data.append(float(car_distance1))
        car_velocity1 = math.sqrt((vehicle_setvelo1[i].get_velocity().x) ** 2.00 + (vehicle_setvelo1[i].get_velocity().y) ** 2.00)
        data.append(car_velocity1 * 18/5)
        if car_distance1 < 100 and car_distance1 > 0:
            result.append(data)
        else:
            data = []

    for i in range(len(vehicle_setvelo2)) :
        data =[]
        car_position2 = vehicle_setvelo2[i].get_location().x
        data.append(vehicle_setvelo2[i].id)
        data.append(car_ID[car_lane2[i]])
        car_distance2 = merge_position_x - car_position2 - car_length[vehicle_actor2[i]]/2
        data.append(float(car_distance2))
        car_velocity2 = math.sqrt((vehicle_setvelo2[i].get_velocity().x) ** 2.00 + (vehicle_setvelo2[i].get_velocity().y) ** 2.00)
        data.append(car_velocity2 * 18/5)
        if car_distance2 < 100 and car_distance2 > 0:
            result.append(data)
        else:
            data = []

def sort_data(): ### ID vehicle verion
    result.sort(key=lambda x: x[1])
    for data in result:
        all_data = []
        for i in range(len(data)):
            all_data.append(str(data[i]))
        output_sensor.append(",".join(all_data))

def control_car():
    global result, output_sensor, timeout
    while True:
        Select_Case()
        time.sleep(1)
        Keyword = "Start"
        pub.publish(Keyword)
        print(Keyword)
        start_time = time.time()
        spawn()
        set_velocity()
        while True:
            st = time.time()
            get_position()
            sort_data()
            pub.publish(";".join(str(i) for i in output_sensor))
            result = []
            output_sensor = []
            while True:
                if time.time()-st >= 0.1:break
                if time.time() - start_time >= timeout:
                    destroy()
                    break
            time.sleep(2)
            break
control_car()