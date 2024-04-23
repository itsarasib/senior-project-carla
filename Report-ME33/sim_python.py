import matplotlib.pyplot as plt
import time
# Input
TD = 0.035 # Average Data Delay [s]
CT = 0.5 # TTC between T2 and other car
LB = 6 # Distance after Confict Point
V_cornering = 10 # T2 Max Conering
a = 0.45 # T2 Acceleration
ba = 2.86 # T2 Full Brake Acceleration
TM = 100 # Max Time for Simulation
dt = 0.001 # Time interval
RS = 1/dt # Simulation Resolution
t = [0] # Time Data
Car_Length = [["t2",4.3],["bike",2],["car",5],["truck",7]] # Length of Car for each type
Car_Type = [i[0] for i in Car_Length] # Type of car in int format(eazy to index)
# Car_Data_Format = [TTC1[s],TTC2[s],TTC3[s],ID[int],TYPE[int],[S[m]],V[km/h],[Time[s]]] T2(Acceleration[m/s^2],Time[s],Case[int])
# T2_Data_Format = [TTC1[s],TTC2[s],TTC3[s],ID[int],TYPE[int],[S[m]],[V[km/h]],[Acceleration[m/s^2]],[Time[s]],[Case[int]]]
Car_Data = [[40/(20*5/18), (40+Car_Length[0][1])/(20*5/18),(40+LB+Car_Length[0][1])/(20*5/18), 0, 0, [40], [20], [0], [0], [0]]]
Car_ID = {0} # Car ID for Multi_Input() (eazy to known its have or not)
Car_temp = [] # Car Data for Car that went off Confict Area
Car_Plot = [] # Merged Data of Car that went off Confict Area
T2S = 0 # T2 State

def Multi_Input(text):
    text = text.replace(" ","").lower().split(";")
    print(text)
    for i in text:
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

def TC():# Calulate TTC1/2/3 in Car_Data
    for i in Car_Data:
        if i[3] == 0:
            if T2S == 2:
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
    global T2S
    temp = []
    for i in Car_Data:
        if i[4] == 0:
            if i[6][-1] + (accel*tpass*18/5) < 0:
                tpass2 = i[6][-1]*(5/18)/accel
                i[5].append(i[5][-1] - (i[6][-1]*tpass2*5/18 + (0.5*accel*(tpass2**2))))
            else:
                i[5].append(i[5][-1] - (i[6][-1]*tpass*5/18 + (0.5*accel*(tpass**2))))
            i[6].append(VL(i[6][-1] + (accel*tpass*18/5)))
            i[8].append(nt)
            if i[6][-1] == 0:T2S = 2
            else:# Backup Car Data that left Confict Area
                i[5].append(i[5][-1] - (i[6]*(tpass)*5/18))
                i[7].append(nt)
            if i[5][-1] < -LB:
                temp.append(i)
                Car_temp.append(i)
                Car_ID.discard(i[3])
    if len(temp) != 0:# Remove Car that left Confict Area
        for i in temp:
            Car_Data.remove(i)
    TC()
    Car_Data.sort() #Sort Car_Data by TTC1

def T2D():# Decision Making System
    global T2S,ba
    T2I = [i[4] for i in Car_Data].index(0)
    #T2 Stopped
    if Car_Data[[j[4] for j in Car_Data].index(0)][6][-1] > V_cornering:return [-a,-1]
    elif T2S == 2:
        if T2I == 0:
            if len(Car_Data) == 1:
                return [4*a,7]
            elif Car_Data[T2I+1][0] > 2.3*1.2:
                return [4*a,7]
            else:
                return [0,6]
        #T2 Braking
        elif T2S == 1:
            return [-ba,5]
        else:
            #Accelaration after pass merge point
            if len(Car_Data) == 1 and Car_Data[[j[4] for j in Car_Data].index(0)][6][-1] < V_cornering - 0.1:
                return [4*a,8]
            #Full Brake
            # T2 hit Car
            if Car_Data[T2I][0] <= 1.4 and T2I > 0 and Car_Data[T2I][0] <= Car_Data[T2I-1][1] + CT:
                ba = Car_Data[T2I][5][-1]/1.4
                T2S = 1
                return [-ba,3]
            # Car hit T2
            elif Car_Data[T2I][0] <= 1.4 and T2I < len(Car_Data)-1 and Car_Data[T2I+1][0] <= Car_Data[T2I][2] + CT:
                ba = Car_Data[T2I][5][-1]/1.4
                T2S = 1
                return [-ba,4]
            #Slow Down
            # T2 hit Car
            elif T2I > 0 and Car_Data[T2I][0] <= Car_Data[T2I-1][1] + CT:
                return [-a,1]
            # Car hit T2
            elif T2I < len(Car_Data)-1 and Car_Data[T2I+1][0] <= Car_Data[T2I][2] + CT:
                return [-a,2]
            else:
                return [0,0]
            
def SIM():# Simulation
    Multi_Input(input("Input Car Data: "))
    for i in range(0,int(TM*RS)+1):
        t.append(i/RS)
        [A,C] = T2D()
        # Collect T2 Data (Acceleration and Case)
        Car_Data[[j[4] for j in Car_Data].index(0)][7].append(A)
        Car_Data[[j[4] for j in Car_Data].index(0)][9].append(C)
        SVC(A,t[-1]-t[-2],i/RS)
        # Stop Simulation if T2 left Confict Area
        if Car_Data[[j[4] for j in Car_Data].index(0)][5][-1] < -LB:
            print(t[-1])
            break

def Plot():# Plot T2 Data
    fig1, T2 = plt.subplots(2,2,figsize=(2*5,2*4))
    T2[0,0].plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] for j in Car_Data].index(0)][5])
    T2[0,0].set_xlim(0, t[-1])
    T2[0,0].set_xlabel('Time [s]')
    T2[0,0].set_ylabel('Distance [m]')
    T2[0,0].grid(True)
    T2[0,1].plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] for j in Car_Data].index(0)][6])
    T2[0,1].set_xlim(0, t[-1])
    T2[0,1].set_xlabel('Time [s]')
    T2[0,1].set_ylabel('Velocity [km/h]')
    T2[0,1].grid(True)
    T2[1,0].plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] for j in Car_Data].index(0)][7])
    T2[1,0].set_xlim(0, t[-1])
    T2[1,0].set_xlabel('Time [s]')
    T2[1,0].set_ylabel('Acceleration [m/s^2]')
    T2[1,0].grid(True)
    T2[1,1].plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] for j in Car_Data].index(0)][9])
    T2[1,1].set_xlim(0, t[-1])
    T2[1,1].set_xlabel('Time [s]')
    T2[1,1].set_ylabel('Case')
    T2[1,1].grid(True)
    fig1.suptitle("T2")

def CTM():# Merge Car Data from Car_Data and Car_Temp into Car_Plot
    CTID = {i[3] for i in Car_temp}
    for id in CTID:
        stemp,ttemp = [],[]
        for i in Car_temp:
            if i[3] == id:
                stemp = stemp + [s for s in i[5]]
                ttemp = ttemp + [s for s in i[7]]
        Car_Plot.append([id,stemp,ttemp])
    return

def MCPlot():# Plot all Car Data
    CTM()
    plt.figure()
    plt.plot(Car_Data[[j[4] for j in Car_Data].index(0)][8], Car_Data[[j[4] for j in Car_Data].index(0)][5])
    for i in Car_Plot:
        plt.plot(i[2],i[1])
    for j in Car_Data:
        if j[4] != 0:plt.plot(j[7],j[5])
    plt.xlim(0, t[-1])
    plt.xlabel("Time [s]")
    plt.ylabel("Distance [m]")
    plt.title("Car")
    plt.grid()
    plt.show()
    return

SIM()
Plot()
MCPlot()

# Example of Input: 1,car,65,10;2,car,30,10