import pandas as pd

filename = '2019-05-18-16-05-31_data_telemetry.csv'

def calculateSpeed(t0, t1, a0, a1):
    return (a1-a0)/((t1-t0)/1000.0)

my_csv = pd.read_csv(filename, usecols=['Time', 'Altitude'])
times = my_csv['Time'].tolist()
alts = my_csv['Altitude'].tolist()

time_speed = []  # List of dicts
for num in range(len(times)):
        if(times[num]==0 or times[num-1]==0): continue
        time_speed.append(
                {'time': times[num],'speed': calculateSpeed(times[num-1], times[num], alts[num-1], alts[num])})




