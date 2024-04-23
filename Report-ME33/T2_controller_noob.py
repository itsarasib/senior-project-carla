from __future__ import print_function
import argparse
import numpy.random as random
import re
import os
import sys
import carla
from agents.navigation.constant_velocity_agent import ConstantVelocityAgent
from agents.tools import misc as tool

import time
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg._String import String
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(10.0)
world = client.get_world()
waypoints = client.get_world().get_map().generate_waypoints(distance=1.0)

Velocity = 20
spawn_point = [waypoint for waypoint in waypoints if waypoint.road_id == 8][-
17].transform
destination = [waypoint for waypoint in waypoints if waypoint.road_id ==
741][5].transform.location
predestination = [waypoint for waypoint in waypoints if waypoint.road_id == 45][-
5].transform.location
stopdestination = [waypoint for waypoint in waypoints if waypoint.road_id ==
741][32].transform.location

# ==============================================================================
# -- T2 Decision functions -----------------------------------------------------
# ==============================================================================
TD = 0.035 # Average Data Delay [s]
CT = 0.5 # TTC between T2 and other car
LB = 6 # Distance after Confict Point 
V_cornering = 10 # T2 Max Conering 
a = 0.45 # T2 Acceleration
ba = 0.8*9.81 # T2 Full Brake Acceleration
t = [0] # Time Data

tt = 0 # Total Time
TM = 100 # Max Time for Simulation
dt = 0.001 # Time interval
RS = 1/dt # Simulation Resolution
IF = 10 # Input Frequency [Hz]
Car_Length = [["t2",4.3],["bike",2],["car",5],["truck",7]] # Length of Car for each type
Car_Type = [i[0] for i in Car_Length]
# Car_Data_Format = [TTC1[s],TTC2[s],TTC3[s],ID[int],TYPE[int],S[m],V[km/hr]] T2(Acceleration[m/s^2],Time[s],Case[int])
Car_Data = [[40/(20*5/18), 
(40+Car_Length[0][1])/(20*5/18),(40+LB+Car_Length[0][1])/(20*5/18), 0, 0, [40], 
[20], [0], [0], [0]]]
Car_ID = {0}
Car_temp = [] # Car Data for Car that went off Confict Area
Car_Plot = [] # Merged Data of Car that went off Confict Area
T2S = 0 # T2 State
CFA = 2.3*1.2
T2_Carla = [[],[]]
Keyword = "Wait"

def Multi_Input(text):
    global Car_Data, Car_ID
    text = text.replace(" ","").lower().split(";")
    # print(text)
    for i in text:
        if len(i.split(",")) == 4:
            id,ct,cs,cv = i.split(",")
            if ct not in Car_Type:# Check Input Car Type
                print("Unknown Car Type")
# Change Input Data Format to Car_Data Format
            else:
                id,cs,cv = int(id),float(cs),float(cv)
                if id in Car_ID:
                    for j in range(0,len(Car_Data)):
                        if id == Car_Data[j][3]:
                            Car_temp.append(Car_Data[j])
                            Car_Data.pop(j)
                            Car_ID.discard(id)
                            break
                ttc1 = cs/(cv*5/18)
                for k in range(0,len(Car_Length)):
                    if ct == Car_Length[k][0]:
                        ct=k
                        break
                ttc2 = (cs+Car_Length[ct][1])/(cv*5/18)
                ttc3 = (cs+LB+Car_Length[ct][1])/(cv*5/18)
                # Save to Car_Data and sort TTC
                Car_Data.append([ttc1,ttc2,ttc3,id,ct,[cs],cv,[t[-1]]])
                Car_Data.sort()
                Car_ID.add(id)

def TC():# Calulate TTC1 and TTC2 in Car_Data
    for i in Car_Data:
        if i[3] == 0:
            if i[6][-1] == 0:
                i[0] = 2.3*1.2
                i[1] = 2.3*1.2
                i[2] = 2.3*1.2
            else:
                i[0] = i[5][-1]/(i[6][-1]*5/18)
                i[1] = (i[5][-1]+Car_Length[i[4]][1])/(i[6][-1]*5/18)
                i[2] = (i[5][-1]+LB+Car_Length[i[4]][1])/(i[6][-1]*5/18)
        else:
            i[0] = i[5][-1]/(i[6]*5/18)
            i[1] = (i[5][-1]+Car_Length[i[4]][1])/(i[6]*5/18)
            i[2] = (i[5][-1]+LB+Car_Length[i[4]][1])/(i[6]*5/18)

def VL(v):# Set Velocity Limit
    v = max(0,v)
    v = min(20,v)
    return v  

def SVC(accel,tpass,nt):# Calulate Position and Velocity in Car_Data
    temp = []
    for i in Car_Data:
        if i[4] == 0:
            if i[6][-1] + (accel*tpass*18/5) < 0:
                tpass2 = i[6][-1]*(5/18)/accel
                i[5].append(i[5][-1] - (i[6][-1]*tpass2*5/18 +
(0.5*accel*(tpass2**2))))
            else:
                i[5].append(i[5][-1] - (i[6][-1]*tpass*5/18 +
(0.5*accel*(tpass**2))))
            i[6].append(VL(i[6][-1] + (accel*tpass*18/5)))
            i[8].append(nt)
        else:# Backup Car Data that left Confict Area
            i[5].append(i[5][-1] - (i[6]*(tpass)*5/18))
            i[7].append(nt)
            if i[5][-1] < -4*LB:
                temp.append(i)
                Car_temp.append(i)
                Car_ID.discard(i[3])
    if len(temp) != 0:# Remove Car that left Confict Area
        for i in temp:
            Car_Data.remove(i)
    TC()
    Car_Data.sort() #Sort Car_Data by TTC1

def T2D(): #Decison Making System
    global T2S
    if T2S == 0:
        return [-0.4,-1]
    elif T2S == 1:
        for i in Car_Data:
            if i[4] != 0:
                if ((i[5][-1] > -LB) and (i[5][-1] <= 0)) or ((i[0] < CFA) and
(i[0] > 0)):return [0,0]
        T2S = 2
        return [0,0]
    elif T2S == 2:
        if Car_Data[[j[4] for j in Car_Data].index(0)][6][-1] < V_cornering:return [0.9,1]
        else:return [0,2]

# ==============================================================================
# -- Plot functions ------------------------------------------------------------
# ==============================================================================

def T2Plot():
    fig1, T2 = plt.subplots(2,2,figsize=(2*5,2*4))
    T2[0,0].plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] 
for j in Car_Data].index(0)][5])
    T2[0,0].set_xlim(0, t[-1])
    T2[0,0].set_xlabel('Time [s]')
    T2[0,0].set_ylabel('Distance [m]')
    T2[0,0].grid(True)

    T2[0,1].plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] 
for j in Car_Data].index(0)][6],T2_Carla[0],T2_Carla[1])
    T2[0,1].set_xlim(0, t[-1])
    T2[0,1].set_xlabel('Time [s]')
    T2[0,1].set_ylabel('Velocity [km/hr]')
    T2[0,1].grid(True)
    T2[1,0].plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] 
for j in Car_Data].index(0)][7])
    T2[1,0].set_xlim(0, t[-1])
    T2[1,0].set_xlabel('Time [s]')
    T2[1,0].set_ylabel('Acceleration [m/s^2]')
    T2[1,0].grid(True)

    T2[1,1].plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] 
for j in Car_Data].index(0)][9])
    T2[1,1].set_xlim(0, t[-1])
    T2[1,1].set_xlabel('Time [s]')
    T2[1,1].set_ylabel('Case')
    T2[1,1].grid(True)

    fig1.suptitle("T2")
    return

def VSPlot():
    fig, ax = plt.subplots()
    twin1 = ax.twinx()
    p1, = ax.plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] 
for j in Car_Data].index(0)][5], "C0", label="Distance")
    p2, = twin1.plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], 
Car_Data[[j[4] for j in Car_Data].index(0)][6], "C1", label="Velocity")
    
    ax.set(xlim=(0, t[-1]), xlabel="Time [s]", ylabel="Distance [m]")
    twin1.set(ylim=(0, 20),ylabel="Velocity [km/hr]")

    ax.yaxis.label.set_color(p1.get_color())
    twin1.yaxis.label.set_color(p2.get_color())

    ax.tick_params(axis='y', colors=p1.get_color())
    twin1.tick_params(axis='y', colors=p2.get_color())

    plt.title("T2 Distance & Velocity")
    ax.legend(handles=[p1, p2])
    ax.grid(True)

    return

def CTM():
    CTID = {i[3] for i in Car_temp}
    for id in CTID:
        stemp,ttemp = [],[]
        for i in Car_temp:
            if i[3] == id:
                stemp = stemp + [s for s in i[5]]
                ttemp = ttemp + [s for s in i[7]]
        Car_Plot.append([id,stemp,ttemp])
    return

def MCPlot():
    CTM()
    plt.figure()
    plt.plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] for j
in Car_Data].index(0)][5])
    for i in Car_Plot:
        plt.plot(i[2],i[1])
    for j in Car_Data:
        if j[4] != 0:plt.plot(j[7],j[5])
    plt.xlim(0, t[-1])
    plt.ylim(Car_Data[[j[4] for j in Car_Data].index(0)][5][-1],100)
    plt.xlabel("Time [s]")
    plt.ylabel("Distance [m]")
    plt.title("All Car Distance")
    plt.grid()
    return

# ==============================================================================
# -- ROS functions -------------------------------------------------------------
# ==============================================================================

def callback(Data):
    global Keyword
    d = Data.data
    if d == "Start":Keyword = "Start"
    elif Keyword == "Start":Multi_Input(d)

def Recieve():
    rospy.init_node('T2_Controller')
    rospy.Subscriber("ch_carla", String, callback)

# ==============================================================================
# -- Global functions ----------------------------------------------------------
# ==============================================================================

def draw_waypoints(waypoints, road_id=None, life_time=50.0):
    for waypoint in waypoints:
        if(waypoint.road_id == road_id):
            world.debug.draw_string(waypoint.transform.location, 'O', 
    draw_shadow=False, color=carla.Color(r=0, g=255, b=0), life_time=life_time, 
    persistent_lines=True)
            
def find_weather_presets():
    """Method to find weather presets"""
    rgx = re.compile('.+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)')
    def name(x): return ' '.join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match('[A-Z].+', x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]

def get_actor_blueprints(world, filter, generation):
    bps = world.get_blueprint_library().filter(filter)

    if generation.lower() == "all":
        return bps
    
    # If the filter returns only one bp, we assume that this one needed
    # and therefore, we ignore the generation
    if len(bps) == 1:
        return bps
    
    try:
        int_generation = int(generation)
        # Check if generation is in available generations
        if int_generation in [1, 2]:
            bps = [x for x in bps if int(x.get_attribute('generation')) ==int_generation]
            return bps
        else:
            print(" Warning! Actor Generation is not valid. No actor will be spawned.")
            return []
    except:
        print(" Warning! Actor Generation is not valid. No actor will be spawned.")
        return []
    
# ==============================================================================
# -- World ---------------------------------------------------------------
# ==============================================================================

class World(object):
    """ Class representing the surrounding environment """
    def __init__(self, carla_world, args):
        """Constructor method"""
        self._args = args
        self.world = carla_world
        try:
            self.map = self.world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print(' The server could not send the OpenDRIVE (.xodr) file:')
            print(' Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self.hud = None
        self.player = None
        self.collision_sensor = None
        self.lane_invasion_sensor = None
        self.gnss_sensor = None
        self.camera_manager = None
        self._weather_presets = find_weather_presets()
        self._weather_index = 0
        self._actor_filter = args.filter
        self._actor_generation = args.generation
        self.restart(args)
        self.recording_enabled = False
        self.recording_start = 0

    def restart(self, args):
        # Get a random blueprint.
        blueprint = random.choice(get_actor_blueprints(self.world, 
self._actor_filter, self._actor_generation))
        
        # Spawn the player.
        if self.player is not None:
            spawn_point.location.z += 0.1
            self.destroy()
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)
        while self.player is None:
            if not self.map.get_spawn_points():
                print('There are no spawn points available in your map/town.')
                print('Please add some Vehicle Spawn Point to your UE4 scene.')
                sys.exit(1)
            spawn_point.location.z += 0.1
            self.player = self.world.try_spawn_actor(blueprint, spawn_point)
            self.modify_vehicle_physics(self.player)

        if self._args.sync:self.world.tick()
        else:self.world.wait_for_tick()

    def modify_vehicle_physics(self, actor):
        #If actor is not a vehicle, we cannot use the physics control
        try:
            physics_control = actor.get_physics_control()
            physics_control.use_sweep_wheel_collision = True
            actor.apply_physics_control(physics_control)
        except Exception:
            pass

    def destroy(self):
        """Destroys all actors"""
        actors = [
            self.player]
        for actor in actors:
            if actor is not None:
                actor.destroy()

    def game_loop(args):
        global tt, Car_Data, Car_ID, t, Velocity, Keyword, T2S
        """
        Main loop of the simulation. It handles updating all the HUD information,
        ticking the agent and, if needed, the world.
        """
        world = None

        try:
            if args.seed:
                random.seed(args.seed)

            traffic_manager = client.get_trafficmanager()
            sim_world = client.get_world()

            if args.sync:
                settings = sim_world.get_settings()
                settings.synchronous_mode = True
                # settings.fixed_delta_seconds = 0.05
                sim_world.apply_settings(settings)

                traffic_manager.set_synchronous_mode(False)
            world = World(client.get_world(), args)

            print("Wait for Start!")
            while True:
                if Keyword == "Start":break
            print("Started")

            if args.agent == "Constant":
                agent = ConstantVelocityAgent(world.player, 20)
                ground_loc = world.world.ground_projection(world.player.get_location(), 5)
                if ground_loc:
                    world.player.set_location(ground_loc.location + carla.Location(z=0.01))
                agent.follow_speed_limits(False)

            # Set the agent destination
            agent.set_destination(predestination)

            # T2 go to predestination
            while True:
                if args.sync:
                    world.world.tick()
                else:
                    world.world.wait_for_tick()

                if agent.done():
                    print("T2 is Subcribed")
                    break

                control = agent.run_step()
                control.manual_gear_shift = False
                world.player.apply_control(control)
            agent.set_destination(stopdestination)

            # T2 go to stopdestination
            lt = 0
            while True:
                st = time.time()
                if lt != 0:
                    tp = time.time()-lt
                else:
                    tp = 0
                lt = time.time()
                tt += tp
                t.append(tt)

                [A,C] = T2D()
                Car_Data[[j[4] for j in Car_Data].index(0)][7].append(A)
                Car_Data[[j[4] for j in Car_Data].index(0)][9].append(C)
                SVC(A,tp,tt)
                Velocity = Car_Data[[j[4] for j in Car_Data].index(0)][6][-1]

                agent.set_target_speed(Velocity)
                velo = tool.get_speed(world.player)
                T2_Carla[0].append(tt)
                T2_Carla[1].append(velo)

                os.system('cls')
                print("T2 is Subcribed")
                print("T2D Case:\t",Car_Data[[j[4] for j in Car_Data].index(0)][9][-1])
                print("T2D Velocity:\t",Velocity)
                print("Carla Velocity:\t",velo)

                if args.sync:
                    world.world.tick()
                else:
                    world.world.wait_for_tick()
                if agent.done():
                    print("T2 is Stopped")
                    T2S = 1
                    Car_Data[[j[4] for j in Car_Data].index(0)][6][-1] = 0
                    break

                control = agent.run_step()
                control.manual_gear_shift = False
                world.player.apply_control(control)

                while(True):
                    if time.time()-st >= dt:break
            # T2 Waiting for car to left conflict area
            while True:
                st = time.time()
                if lt != 0:
                    tp = time.time()-lt
                else:
                    tp = 0
                lt = time.time()
                tt += tp
                t.append(tt)

                [A,C] = T2D()
                Car_Data[[j[4] for j in Car_Data].index(0)][7].append(A)
                Car_Data[[j[4] for j in Car_Data].index(0)][9].append(C)
                SVC(A,tp,tt)
                Velocity = Car_Data[[j[4] for j in Car_Data].index(0)][6][-1]

                velo = tool.get_speed(world.player)
                T2_Carla[0].append(tt)
                T2_Carla[1].append(velo)

                os.system('cls')
                print("T2 is Waiting")
                print("T2D Case:\t",Car_Data[[j[4] for j in Car_Data].index(0)][9][-1])
                print("T2D Velocity:\t",Velocity)
                print("Carla Velocity:\t",velo)
                if args.sync:
                    world.world.tick()
                else:
                    world.world.wait_for_tick()
                if T2S == 2:
                    break

                while(True):
                    if time.time()-st >= dt:break

            agent.set_destination(destination)

            # T2 go to destination
            lt = 0
            while True:
                st = time.time()
                if lt != 0:
                    tp = time.time()-lt
                else:
                    tp = 0
                lt = time.time()
                tt += tp
                t.append(tt)

                [A,C] = T2D()
                Car_Data[[j[4] for j in Car_Data].index(0)][7].append(A)
                Car_Data[[j[4] for j in Car_Data].index(0)][9].append(C)
                SVC(A,tp,tt)
                Velocity = Car_Data[[j[4] for j in Car_Data].index(0)][6][-1]

                agent.set_target_speed(Velocity)
                
                velo = tool.get_speed(world.player)
                T2_Carla[0].append(tt)
                T2_Carla[1].append(velo)

                os.system('cls')
                print("T2 is Merging")
                print("T2D Case:\t",Car_Data[[j[4] for j in Car_Data].index(0)][9][-1])
                print("T2D Velocity:\t",Velocity)
                print("Carla Velocity:\t",velo)

                if args.sync:
                    world.world.tick()
                else:
                    world.world.wait_for_tick()
                if agent.done():
                    print("T2 is Unsubcribed")
                    break

                control = agent.run_step()
                control.manual_gear_shift = False
                world.player.apply_control(control)

                while(True):
                    if time.time()-st >= dt:break
        finally:

            if world is not None:
                settings = world.world.get_settings()
                settings.synchronous_mode = False
                settings.fixed_delta_seconds = None
                world.world.apply_settings(settings)
                world.destroy()
        
    def main():
        """Main method"""

        argparser = argparse.ArgumentParser(
            description='CARLA Automatic Control Client')
        argparser.add_argument(
            '-v', '--verbose',
            action='store_true',
            dest='debug',
            help='Print debug information')
        argparser.add_argument(
            '--host',
            metavar='H',
            default='127.0.0.1',
            help='IP of the host server (default: 127.0.0.1)')
        argparser.add_argument(
            '-p', '--port',
            metavar='P',
            default=2000,
            type=int,
            help='TCP port to listen to (default: 2000)')
        argparser.add_argument(
            '--res',
            metavar='WIDTHxHEIGHT',
            default='640x360',
            help='Window resolution (default: 640x360)')
        argparser.add_argument(
            '--sync',
            action='store_true',
            help='Synchronous mode execution')
        argparser.add_argument(
            '--filter',
            metavar='PATTERN',
            default='vehicle.volkswagen.t2',
            help='Actor filter (default: "vehicle.volkswagen.t2")')
        argparser.add_argument(
            '--generation',
            metavar='G',
            default='2',
            help='restrict to certain actor generation (values: "1","2","All" -default: "2")')
        argparser.add_argument(
            '-l', '--loop',
            action='store_true',
            dest='loop',
            help='Sets a new random destination upon reaching the previous one (default: False)')
        argparser.add_argument(
            "-a", "--agent", type=str,
            choices=["Behavior", "Basic", "Constant"],
            help="select which agent to run",
            default="Constant")
        argparser.add_argument(
            '-b', '--behavior', type=str,
            choices=["cautious", "normal", "aggressive"],
            help='Choose one of the possible agent behaviors (default: normal) ',
            default='normal')
        argparser.add_argument(
            '-s', '--seed',
            help='Set seed for repeating executions (default: None)',
            default=None,
            type=int)
        
        args = argparser.parse_args()

        args.width, args.height = [int(x) for x in args.res.split('x')]

        try:
            game_loop(args)

        except KeyboardInterrupt:
            print('\nCancelled by user. Bye!')

    if __name__ == '__main__':
        try:
            Recieve()
            main()
            time.sleep(5)
            T2Plot()
            VSPlot()
            MCPlot()
            plt.show()
        except rospy.ROSInterruptException:
            pass