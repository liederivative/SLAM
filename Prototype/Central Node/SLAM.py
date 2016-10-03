import numpy as np
from robot1Pi import robot
from pprint import pprint as prnt
import random
from serialChannel import serialChannel
import atexit

import sys
#import pdb
import copy
from time import clock
from time import sleep

ini_pose = dict(x=0,y=0,orientation=0) #dict(x=15,y=-20,orientation=0)
ini_noise = dict(new_f_noise=30,
                 new_t_noise=(2*np.pi/180),
                 new_d_noise=1200,
                 new_a_noise=2) # forward,turn,distance,angle
# World param
landmarks = []
# landmarks = [[random.randint(-50,60),random.randint(-45,55)] for x in range(50)]

num_landmarks = len(landmarks)
seen = [False for row in range(num_landmarks)]
world = dict(world_size = 100.0,
             measurement_range = 40.0,
             num_landmarks = num_landmarks,
             angle_vision = np.pi/4,
             world_landmarks = landmarks)

Vxy = 0#5.0 # velocity
Wxy = 0#0.2 # theta

data = []

# Simulation Param
N = 10*10 # number of particles
Steps = 500 # number of steps to run


# init robot
robotCar = robot()
robotCar.set(**ini_pose)
robotCar.set_world(**world)
robotCar.init_seen(num_landmarks)
robotCar.set_noise(**ini_noise)

#init particles
p = []
for i in range(N):
    p.append(robot())
for i in p:
    i.set(**ini_pose)
    i.set_noise(**ini_noise)
    i.set_world(**world)
    i.init_seen(num_landmarks)
    i.weight = 1.0/N


# run simulation
#f = open('DATA_SLAM.txt','w')
#############################
######## Aux function ######
def new_particle(list_index,p):
    p3 = []

    for index in list_index:
        t = copy.deepcopy(p[index])
        t.weight = 1.0/N
        p3.append(t)
    return p3
#run()
#f.close()

#########################################################################
 #   Set Channel

# channel = serialChannel(115200,'COM3')
# import time
# channel.setDaemon(True)
# channel.start()
# atexit.register(channel.exit_handler)
# time.sleep(2) # await for serial port
filename = sys.argv[1]
# def main(argv):
#     global filename
#     filename = str(argv)
#     print filename
import process
if filename:
    f = file(filename,"r")
#f = file("exp02_14.txt","r") # 132,121,12,41,31,42  EXP2 132,121,12,-41,42,31
# exp01_26 - 25 - 24(just 3) - 22(skewed 6)** - 20 (skewed 5) - 17(uncert 5) **
# exp02_7 - 8 - 14(milk) -
    static_data = f.read()
    static_data = process.give_data(static_data)
    f.close()
#########################################################################
# Set up Graphics
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg

#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

win = pg.GraphicsWindow(title="RealTime Visualization FastSLAM 1.0")
win.resize(500,500)
win.setWindowTitle('FastSLAM 1.0')
label = pg.LabelItem(justify = "right")
win.addItem(label)
# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

#p1 = win.addPlot(title="Updating plot")

p1 = win.addPlot(row = 1, col = 0)


#p1.setXRange(-50,60)
#p1.setYRange(-45,55)
p1.setXRange(-0.1*1000,0.5*1000)
p1.setYRange(-0.1*1000,0.5*1000)

vb = p1.vb
def getCoord(evt):
    # mousePointX = p1.vb.mapSceneToView(event[0]).x()
    # mousePointY = p1.vb.mapSceneToView(event[1]).y()
    mousePoint = vb.mapSceneToView(evt[0])
    label.setText("<span style='font-size: 14pt; color: white'> x = %0.2f, <span style='color: white'> y = %0.2f</span>"\
     % (mousePoint.x(), mousePoint.y()))

proxy = pg.SignalProxy(p1.scene().sigMouseMoved, rateLimit=60, slot=getCoord)

# landmarks view
# landmarks_plot = p1.plot(pen=None ,symbol='d')
# # l_points  = np.array([[40.0, -20.0],[50,15],[40.0, 40.0],[10,45],\
# #               [-20.0, 40.0],[-30,10],[-20.0, -20.0],\
# #               [10,-30],[-40,-40]])
# l_points = np.stack(landmarks)
# #data = np.random.normal(size=(10,1000))
#
# landmarks_plot.setData(l_points)

# Real motion view
motion = p1.plot(pen='y')
pen = pg.mkPen(cosmetic=False, width=4.5, color='r')

# particle view
particle = []
for c in range(N):
    particle.append(p1.plot(pen=None,symbol='+',brush=pg.mkBrush(255, 255, 255, 120)))
group_particles = p1.plot(pen='g',symbol='o')
track_particle = p1.plot(pen='r')

ptr = 1
n= 0
data = []
data_particles = []
connect_dots = []
p1.enableAutoRange('xy', False)
store =[]
node = {}
def update():
    global ptr, p,robotCar,Vxy,Wxy,n
    if n < Steps:
        tnt = clock()
        # node = channel.get()
        node = {'odometry':static_data['odometry'][n],'control':static_data['control'][n],\
        'distance':static_data['distance'][n]}
        store.append(node)
        #print node
        n+=1
        #robotCar =
        Wxy = node['control'][1]
        Vxy = node['control'][0]
        # robotCar.set(node['odometry'][0], node['odometry'][1],node['odometry'][2]) # move robot
        #print "\n[X: ",robotCar.x,",Y: ",robotCar.y,"]"

        print"node odometry", node['odometry']
        data.append([ node['odometry'][0], node['odometry'][1] ])
        # print [ node['odometry'][0], node['odometry'][1] ]
        #data.append([robotCar.x,robotCar.y])
        motion.setData(np.array(data))
        #f.write(str(robotCar.x)+" "+str(robotCar.y)+"\n")
        # Z = robotCar.sense() # sense enviroment
        Z = []
        if node['distance']:
            # print node['distance'][0]
            for x in node['distance']:
                #print node['distance']
                signature = x[2]
                if signature not in landmarks:
                    landmarks.append(signature) #append landmark locations
                i = landmarks.index(signature)
                Z.append( robotCar.sense(x,i) )
        # for i_ in range(len(Z)):
        #     seen[Z[i_][0]] = True

        #predict Z observation to landmarks
##        print "Z",Z
        # Compute
        w = []
        p3 = []
        do_resample = False
        for i_p,p_ in enumerate(p):
            if not(node['control'][0] == 0.0) :
                xt_1 = [node['odometry'][0],node['odometry'][1],node['odometry'][2]]
                # print "sample: ",xt_1
                p[i_p].move(Wxy,Vxy,pos=xt_1) # move particles
                # print "moving p:",i_p, "coord",p[i_p]
            # print "Z",Z
            if Z:

                p[i_p].update_measurement(Z) # update mean,cov of landmarks

                do_resample = True



        w = [p[i].get_weight() for i in range(len(p))]
        normalizer = sum(w)

        # print "normalizer, sum(w)", normalizer
        # print w
        wn = map(lambda x: x/normalizer, w) #normalized weights
        for e in range(len(p)):
            p[e].weight = p[e].weight/normalizer
        #     w[e] = w[e]/normalizer
        wv = np.array(wn) # vector form
        # print [p[i].get_weight() for i in range(len(p))]
        neff = 1/sum(wv**2)
##        print "neff",neff
        # print "do_resample", do_resample
        # print "neff < N/2", neff < N/2.0
        if do_resample and (neff < N*.75): #and seen.count(True) != len(landmarks)-1:
##            print "doing resampling"

            # for i in range(len(p)):
            #     # apply weights to particles according readings
            #     w.append(p[i].weight)
            w = [round(x,6) for x in w]
            # low-variance resample
            index = int(random.random() * N)
            u = 0.0
            list_index = []
            mw = max(w)
            for i in range(N):
                u += random.random() * 2 * mw
                while u > w[index]:
                    u -= w[index]
                    index = (index + 1) % N
                list_index.append(index)
##            print "get_index"
            p4 = new_particle(list_index,p)

            p = p4


        p_data = np.array([[i.x,i.y] for i in p])
##        #print "p_data",p_data
        mean_p_data_x = np.mean(p_data.T[0])
        mean_p_data_y = np.mean(p_data.T[1])
        data_particles.append([mean_p_data_x,mean_p_data_y])
        landamarks_per_particle = np.array([])
        track_particle.setData(np.array(data_particles))
        for i,y in enumerate(p):
           landamarks_per_particle = np.array([ [ l[1].T[0][0],l[1].T[0][1] ]  for l in y.landmarks])

           particle[i].setData(landamarks_per_particle)
        #     if (node['distance'][0] != 0.0):
                # particle[i].setData(np.array([node['distance'][0]/10]))
        # print node['distance'][0]
        # if channel.get()['distance']:
        #     store.append([ channel.get()['distance'][0]/10])
            # particle[i].setData(np.array(store))#, channel.get()['distance'][1]/10 ])
        group_particles.setData(p_data)#np.array(data_particles))

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)



## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':

    print(sys.argv)
    # main(sys.argv[1])
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
