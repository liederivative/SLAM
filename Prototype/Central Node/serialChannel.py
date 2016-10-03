# System modules
import Queue
from threading import Thread
import serial
from sys import exit
class serialChannel(Thread):
    def __init__(self, BAUDRATE, PORT):
        Thread.__init__(self,target=self.readChannel)
        try:
            self.ser = serial.Serial(
            port =PORT,
            baudrate = BAUDRATE,
            parity = serial.PARITY_NONE,
            timeout=0
            )
        except Exception as inst:
            print "Port already open or disconnected."
        self.q = Queue.Queue()
        self.data = {'odometry':[],'control':[],'distance':[]}
    def readChannel(self):
        cache = []
        while True:
            read = ''
            try:
                read = self.ser.read() if self.ser.is_open else exit()
            except Exception as inst:
                print "Error on Channel.",inst
                self.ser = False
                exit()

            for line in read:#self.ser.read():
                cache.append(line)
                if line == '\n':
                    msg = ''.join(character for character in cache)
                    # if "Odometry" in msg:
                    #     odometry = self.processString(msg,"Odometry")
                    #     self.data['odometry'] = odometry
                    if "distance" in msg:
                        distance = self.processString(msg,"distance")
                        self.data['distance'] = distance
                    # else:
                    #     self.data['distance'] = []
                    if "control" in msg:
                        msg = msg.split(",")
                        control = msg[0].replace("control: ","").split("|")
                        odometry = msg[1].replace("Odometry: ","").\
                                   replace('\r\n','').split("|")
                        self.data['control'] = map(float,control)
                        self.data['odometry'] = map(float,odometry)
                    self.q.put(self.data)
                    self.q.task_done()
                    cache = []
                    break
    def get(self):
        item = {}
        try:
            item = self.q.get()
        except Queue.Empty:
            print "No element at queue"
            exit()
        return item
    def processString(self, msg, lookupWord):
        phrase = lookupWord + ": "
        pre_process = msg.replace(phrase,"")\
        .replace('\r\n','').split("|")
        pre_process = map(float,pre_process)
        return pre_process
    def exit_handler(self):
        if self.ser:
            self.ser.close()
    def close():
        self.ser.close()
