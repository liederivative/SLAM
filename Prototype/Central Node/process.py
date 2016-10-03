def processString(msg, lookupWord):
    phrase = lookupWord + ": "
    pre_process = msg.replace(phrase,"")\
    .replace('\n','').split("|")
    pre_process = map(float,pre_process)
    return pre_process
def give_data(data):
    r_data = []
    t_data = {'odometry':[],'control':[],'distance':[]}
    read = data.split("\n")
    for line in read:#self.ser.read():
        msg = line
        # if "distance" in msg:
        #     distance = processString(msg,"distance")
        #     t_data['distance'].append(distance)
        # else:
        #     self.data['distance'] = []
        if "control" in msg:
            msg = msg.split(",")

            control = msg[0].replace("control: ","").split("|")
            odometry = msg[1].replace("Odometry: ","").split("|")
            pre_distance = msg[2].replace("distance: ","").replace("\n","").split("||")

            distance = [ map(float,d.split("|")) for d in pre_distance]
            distance.pop(-1)
            t_data['distance'].append(distance)
            t_data['control'].append( map(float,control) )
            t_data['odometry'].append( map(float,odometry) )


    return t_data
