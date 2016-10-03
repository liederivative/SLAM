from serial import *
from Tkinter import *
import ttk
import datetime
import os
os.chdir('/home/pi/Desktop/Python_projects/') # directory holding text files
class GUISLAM(object):
    def __init__(self):
        self.data = []
        self.store = []
        self.serialPort = "/dev/ttyUSB0"
        self.baudRate = 115200
        self.ser = Serial(self.serialPort , self.baudRate, timeout=0, writeTimeout=0) #ensure non-blocking
        # Robot Movement
        self.flag_button = False
        self.jobid = None
        # File variables
        self.f = None
        self.save = False
        self.filename = ''

        #make a TkInter Window
        self.root = Tk()

        self.root.wm_title("GUISLAM")
        # make a scrollbar
        self.scrollbar = Scrollbar(self.root)
        self.scrollbar.grid(row=0,column=1)
        #self.scrollbar.pack(side=RIGHT, fill=Y)
        # make a text box to put the serial output
        self.log = Text ( self.root, width=30, height=30, takefocus=0)
        self.log.grid(rowspan=2,column=0)
        #self.log.pack()
        # attach text box to scrollbar
        self.log.config(yscrollcommand=self.scrollbar.set)
        self.scrollbar.config(command=self.log.yview)
        #make our own buffer
        self.buff=""#0
        #List of files
        self.listbox = Listbox(self.root)
        self.listbox.grid(row=1,column=5)

        #Buttons
        self.bou=Button(self.root,text='UP')

        self.bou.grid(row=0,column=3)

        self.bou1=Button(self.root,text='LEFT')

        self.bou1.grid(row=0,column=2)

        self.bou2=Button(self.root,text='RIGHT')

        self.bou2.grid(row=0,column=4)

        self.save_btn = StringVar()

        self.bou3=Button(self.root,textvariable=self.save_btn, command=self.file)
        self.save_btn.set('SAVE RESULTS')
        self.bou3.grid(row=0,column=5)
        self.bou4=Button(self.root,text='STOP SAVING',state=DISABLED,command=self.stopSaving)
        self.bou4.grid(row=0,column=6)
        self.bou5=Button(self.root,text='PROCESS FILE',state=DISABLED,command=self.processFile)
        self.bou5.grid(row=1,column=6)
        self.bou6=Button(self.root,text='DELETE FILE',state=DISABLED,command=self.delFile)
        self.bou6.grid(row=2,column=6)
        self.bou7=Button(self.root,text='RESET',command=self.reset)
        self.bou7.grid(row=3,column=6)

        self.root.after(100, self.readSerial)
        self.root.protocol("WM_DELETE_WINDOW", self.Intercept)
        #self.root.mainloop()
        self.setListFiles()
    def reset(self):
        self.ser.close()
        self.ser.open()
    def delFile(self):
        selection = self.listbox.get(self.listbox.curselection())
        os.remove(selection)
        self.setListFiles()
    def setListFiles(self):
        self.listbox.delete(0, END)
        listfiles = os.listdir(os.getcwd())
        listfiles = [name for name in listfiles if ("GUISLAM" in name) and (".txt" in name)  ]
        for f in listfiles:
            self.listbox.insert(END,f)
        if listfiles:
            self.bou5.configure(state=NORMAL)
            self.bou6.configure(state=NORMAL)

    def moveForward(self,dir):

        self.flag_button = True
        self.writeSerial("8")
    def move(self,dir):
        print('running')
        self.jobid = self.root.after(100, self.move, dir)
        self.writeSerial(dir)
    def moveStop(self):
         self.root.after_cancel(self.jobid)
    def keydown(self,e):
        if e.char.isdigit():
            # print(e.char)
            self.writeSerial(e.char)
    def keyup(self,e):
        self.moveStop()
    def mainloop(self):
        self.bou.bind('<ButtonPress-1>',lambda event: self.move('8'))
        self.bou.bind('<ButtonRelease-1>',lambda event: self.moveStop())
        self.bou1.bind('<ButtonPress-1>',lambda event: self.move('4'))
        self.bou1.bind('<ButtonRelease-1>',lambda event: self.moveStop())
        self.bou2.bind('<ButtonPress-1>',lambda event: self.move('6'))
        self.bou2.bind('<ButtonRelease-1>',lambda event: self.moveStop())

        self.root.bind('<KeyPress>',self.keydown)
        # self.root.bind('<KeyRelease>',self.keydown)
        self.root.mainloop()
    def file(self):
        self.save_btn.set("SAVING FILE")
        self.bou4.configure(state=NORMAL)
        self.bou3.configure(state=DISABLED)
        t = datetime.datetime.now()
        self.filename = 'GUISLAM_'+str(t.month)+str(t.year)+str(t.day)+'_'+str(t.hour)+str(t.minute)+'.txt'
        self.f = file(self.filename,'w')
        self.save = True
    def stopSaving(self):
        self.save_btn.set("SAVE RESULTS")
        self.bou5.configure(state=NORMAL)
        self.bou3.configure(state=NORMAL)
        self.bou4.configure(state=DISABLED)
        self.save = False
        self.setListFiles()
        self.f.close()
    def processFile(self):
        selection = self.listbox.get(self.listbox.curselection())
        print(selection)
        command = "python SLAM.py "+selection
        os.system(command)
    def Intercept(self):
        try :
            self.ser.close()
            if self.save:
                self.f.close()
        except:
            pass
        self.root.destroy()
    def writeSerial(self,data):

        self.ser.write(data)

    def readSerial(self):
        while True:
            self.c = self.ser.read() # attempt to read a character from Serial
            #was anything read?
            if len(self.c) == 0:
                break
            if self.c == '\r':
                self.c = '' # don't want returns. chuck it

            if self.c == '\n':
                self.buff += "\n" # add the newline to the buffer

                # add the line to the TOP of the log
                self.store.append(self.buff)
                if "control" in self.buff:
                    if self.save:
                        self.f.write(self.buff)
                if "Odometry" in self.buff:
                    self.data = self.buff.replace("Odometry: ","").split("|")
                ####
                self.log.insert('0.0', self.buff)
                self.buff = "" # empty the buffer
            else:
                self.buff += self.c # add to the buffer

        self.root.after(10, self.readSerial) # check serial again soon


t = GUISLAM()
t.mainloop()
