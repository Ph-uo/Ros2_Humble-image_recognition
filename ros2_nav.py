import serial
ser=serial.Serial(port='COM3', baudrate=9600, timeout=1) #timeout thoi gian toi da doc du lieu

def init_pose():
    global vr,vl
    if have_map == True && have_estimate_pose == True:
        navigation()
    else:
        vr=0.1
        vl=-0.1
        ser.write(vl,vr)

def navigation():
    pass

class robot (self):
    def init(self):
        pass

    def pose(self,x,y,theta):
        pass
    
    def position(self,x,y):
        pass

    rot=lambda a:a+0

    
