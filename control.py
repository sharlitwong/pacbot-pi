
import serial
import time

port = "/dev/serial0"
baud = 9600

import struct


class test_mov():
    def __init__(self, m1_s=800,m2_s=1000,m3_s=1000,m4_s=1000):
        self.ser = serial.Serial(port, baud, timeout=1)
        self.m1_speed=m1_s
        self.m2_speed=m2_s
        self.m3_speed=m3_s
        self.m4_speed=m4_s
        self.command_list=["s1","s2","s3","s4","f","b","r","l","s","tr"]
        pass
    
    def send_motors(self,sp1,sp2,sp3,sp4):
        # scale from -1000:1000 to -32767:32767
        def scale(val):
            return int(val * 32767 / 1000)
 #       print("Sent\n")
        data = struct.pack('>hhhh', scale(sp1), scale(sp2), scale(sp3), scale(sp4))
        self.ser.write(data)
        self.ser.write(data)
        print(f"Sent {data}")
    
    def try_error(self, len,command_list, num):
        if(command_list<len):
            return True
        return False
    #left front right back
    def take_command(self):
        print("input command")
        in2=input()
        command_components=in2.split()
        command=command_components[0]
        if(command_components[0] in self.command_list):
            if(command=="s1"):
                try:
                    self.m1_speed=int(command_components[1])
                except Exception as e:
                    print(e)
                
                pass
            elif(command=="s2"):
                try:
                    self.m2_speed=int(command_components[1])
                except Exception as e:
                    print(e)
                pass
            elif(command=="s3"):
                try:
                    self.m3_speed=int(command_components[1])
                except Exception as e:
                    print(e)
                pass
            elif(command=="s4"):
                try:
                    self.m4_speed=int(command_components[1])
                except Exception as e:
                    print(e)
                pass
            elif(command=="f"):
                self.send_motors(self.m1_speed,0,-1*self.m3_speed,0)
                pass
            elif(command=="b"):
                self.send_motors(-1*self.m1_speed,0,self.m3_speed,0)
                pass
            elif(command=="r"): 
                self.send_motors(0,self.m2_speed,0,-1*self.m4_speed)
                pass
            elif(command=="l"):
                self.send_motors(0,-1*self.m2_speed,0,self.m4_speed)
                pass
            elif(command=="s"):
                self.send_motors(0,0,0,0)
                pass
            elif(command=="a"):
                self.send_motors(self.m1_speed,self.m2_speed,self.m3_speed,self.m4_speed)
            elif(command=="tr"):
                self.send_motors(-1*self.m1_speed,self.m2_speed,self.m3_speed,-1*self.m4_speed)
            else:
                print("command not recognized")
    def close(self):
        if 'ser' in locals() and self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    bot_mov=test_mov()
    bot_mov.send_motors(0,0,0,0)
    try:
        while(True):
            bot_mov.take_command()
            pass
    except KeyboardInterrupt as e:

        bot_mov.send_motors(0,0,0,0)
        bot_mov.close()
        exit



