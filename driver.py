import serial
import time,sys
import signal
import os
class Lidar:
    def __init__(self,port):
        self.port = port
        self.serial =None

    def init(self):
        if os.path.exists(self.port):
            baud_rate= 230400
            self.serial = serial.Serial(self.port,baud_rate)
            self.start()
            return True
        else:
            return False

    def start(self):
        cmd = ("b")
        self.serial.write(cmd.encode())

    def stop(self):
        try:
            cmd= ("e")
            self.serial.write(cmd.encode())
            self.serial.close()
        except:
            pass

    def poll(self):
        temp_char = None
        start_count = 0
        got_scan = False
        raw_bytes  =[]
        good_sets = 0
        motor_speed = 0
        index = 0
        while not got_scan :
            raw_bytes[start_count:start_count+1] = self.serial.read(1)
            if start_count == 0:
                if raw_bytes[start_count] == 0xFA:
                    start_count = 1
            elif start_count ==1:
                if raw_bytes[start_count]==0xA0:
                    start_count = 0
                    # read in the rest of the message
                    got_scan = True
                    raw_bytes[2:]=self.serial.read(2518)
                    # scan->angle_min = 0.0;
                    # scan->angle_max = 2.0*M_PI;
                    # scan->angle_increment = (2.0*M_PI/360.0);
                    # scan->range_min = 0.12;
                    # scan->range_max = 3.5;
                    # scan->ranges.resize(360);
                    # scan->intensities.resize(360);
                    for i in range(0,len(raw_bytes),42):
                        if raw_bytes[i] == 0xFA and raw_bytes[i+1] == (0xA0+i/42):
                            good_sets+=1
                            motor_speed += (raw_bytes[i+3] << 8) + raw_bytes[i+2] # accumlate count for avg
                            rpms = (raw_bytes[i+3]<<8|raw_bytes[i+2])/10
                            for j in range(i+4,i+40,6):
                                index = 6*(i/42)+(j-4-i)/6
                                byte0 = raw_bytes[j]
                                byte1 = raw_bytes[j+1]
                                byte2 = raw_bytes[j+2]
                                byte3 = raw_bytes[j+3]

                                #remaining bits are the range in mm
                                intensity = (byte1<<8)+byte0
                                # Last two bytes represent the uncertanty or intensity, might also be pixel area of target...
                                # uint16_t intensity = (byte3 << 8) + byte2;
                                _range = (byte3 << 8) + byte2
                                yield index,_range
                else:
                    start_count = 0


