# import serial
import socket
import time

import threading
from TGTs import *


HOST = 'localhost'
PORT = 2250  # state socket port

# ardu = serial.Serial('/dev/ttyUSB0', 115200)  # Arduino "/dev/ttyACM0"
# time.sleep(3)
update_mission(mision_list,new_point=None)

Target = [-35.35643454149282,  149.16611533539404 ,  584]
loiter_cmd = False

Target_list = []
target_num = 0
target_x = 0
target_y = 0
target_z = 0

tilt, pan = 0 , 0

cmd_enum, cmd_param1, cmd_param2, cmd_param3 = 2 , 0 , 0 , 0
mode = 2
lock_mode = 0

connected = False


def server_handling():
    i = 0
    global lat,lon,elev,tilt,pan,distance,bearing,psi,lat1,lon1,elev1  # send
    global cmd_enum, cmd_param1, cmd_param2, cmd_param3 # receive
    global connected, mode, loiter_cmd, loiter_lat , loiter_lon
    target_num, target_x, target_y, target_z = 0,0,0,0
    connected = False
    while True:
        if not connected:
            try:
                clinet = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                clinet.connect((HOST, PORT))
                connected = True
                print("connected")
            except:
                print("Server is not yet available. Retrying in 3 second...")
                time.sleep(3)


        if connected:
            # itrate through the targets in the target list, sending one target at time to the server 
            if i < len(Target_list):
                target_num, target_x, target_y = Target_list[i][0], Target_list[i][1], Target_list[i][2]
                i+=1
                # print(target_num)
            else:
                i = 0
            
            try:
                clinet.send(
                    f"{round(lat,7)},{round(lon,7)},{round(elev,1)},{round(tilt,1)},{round(pan,1)},{round(distance,1)},{round(bearing,1)},{round(psi,2)},{round(lat1,7)},{round(lon1,7)},{round(elev1,1)},{int(target_num)},{round(target_x,7)},{round(target_y,7)},{round(target_z,1)}#".encode('utf-8'))   
            except:
                connected = False
                print('disconnected')
                time.sleep(1)

            try:
                message = clinet.recv(1024).decode('utf-8')
                data = message.split('#')
            except:
                connected = False
                print('disconnected')
            
            for cmd in data:
                if cmd != '':
                    cmd = cmd.split(',')
                    cmd_enum, cmd_param1, cmd_param2, cmd_param3 = int(cmd[0]), float(cmd[1]), float(cmd[2]), float(cmd[3])

            if cmd_enum == 10:
                Target[0], Target[1], Target[2] = cmd_param1, cmd_param2, cmd_param3
                if Target[2]==0:
                    master.mav.send(mavutil.mavlink.MAVLink_terrain_check_message(int(lat*1e7), int(lon*1e7)))
                    report = master.recv_match(type='TERRAIN_REPORT', blocking=True)
                    if report.lat == int(lat*1e7):
                        Target[2]= report.terrain_height
                        mode = 2
                        print('recived slave command, target elevation : ',elev)
                        cmd_enum = 0
                    else:
                        continue

            if cmd_enum==20:
                print('revied loitering command, loiter point : ',cmd_param1,' ',cmd_param2)
                loiter_cmd = True  # open the command in the progrogram loop
                loiter_lat , loiter_lon = cmd_param1 , cmd_param2
                cmd_enum = 0  # clear the 

threading.Thread(target=server_handling,daemon=True).start()

while True:
    
    # # arduino handling
    # ardu_data = ardu.readline()
    # ardu.flushInput()
    # ardu_data = ardu.readline()
    # ardu_data = (str(ardu_data, 'utf-8')).strip('\r\n')
    # try :
    #     if ardu_data[0] != "!":
    #         continue
    # except : 
    #     continue

    # ardu_data = ardu_data[1:]
    # gimbal = ardu_data.split(',')
    # pan, tilt = float(gimbal[0]) , float(gimbal[1])  

    

    if connected:         # CMD Loop connected
        if 1 <= cmd_enum <= 10:
            # print('CMD_Loop connected')
            mode , tilt_cmd, pan_cmd = cmd_enum, cmd_param1, cmd_param2

        else :
            pass
        

    if loiter_cmd:
        loiter(loiter_lat,loiter_lon)
        loiter_cmd = False

    # if cmd_enum == 10:
    #     Target[0], Target[1], Target[2] = cmd_param1, cmd_param2, cmd_param3
    #     if Target[2]==0:
    #         master.mav.send(mavutil.mavlink.MAVLink_terrain_check_message(int(lat*1e7), int(lon*1e7)))
    #         report = master.recv_match(type='TERRAIN_REPORT', blocking=True)
    #         if report.lat == int(lat*1e7):
    #             Target[2]= report.terrain_height
    #             mode = 2
    #             print(elev,report.lat,int(lat*1e7))
    #         else:
    #             continue


    if mode == 10:
        control_mode = 0
        tilt_cmd, pan_cmd , distance, bearing, psi,lat1,lon1,elev1 = lock(lat,lon,elev)
        
    if mode == 1:  #center
        control_mode = 0
        tilt, pan, tilt_cmd, pan_cmd  = 0, 0, 0, 0
        lat, lon, elev, distance, bearing, psi = 0, 0, 0, 0, 0, 0
        lat1,lon1,elev1=0,0,0


    if mode == 2:  #slave
        control_mode = 0
        lock_mode = 0
        lat, lon, elev = Target[0], Target[1], Target[2] 
        tilt_cmd, pan_cmd , distance, bearing, psi,lat1,lon1,elev1 = lock(lat, lon, elev)

    if mode == 3:  #Desingate
        control_mode = 1  # drive the motors based on given speed
        lat, lon, elev, distance, bearing, psi,lat1,lon1,elev1 = designate(tilt,pan)
        lock_mode = 0


    if mode == 4:  #locke

        if lock_mode == 0:
            lat, lon, elev, distance, bearing, psi,lat1,lon1,elev1 = ranging(tilt, pan)
            Target_list.append([len(Target_list)+1,lat,lon])
            print('Target added !', Target_list)
            lock_mode = 1  # target aqquired

        if lock_mode == 1:
            control_mode = 0  # drive the motors based on given angles
            tilt_cmd, pan_cmd , distance, bearing, psi,lat1,lon1,elev1 = lock(lat, lon, elev)


    # # send commands to arduio controller
    # cmd = (f'{control_mode} ,{tilt_cmd} ,{pan_cmd}')
    # cmd = (cmd+'\r')
    # ardu.write(cmd.encode())
