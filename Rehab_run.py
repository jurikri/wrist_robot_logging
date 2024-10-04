## Packet Structure, Packet Send, Run, Receive Data
from serial import Serial
import keyboard
import threading
import csv
from datetime import datetime
import time
import serial.tools.list_ports
import binascii
import struct
import collections
import time

# 전원 넣을 때 가운데
# 코드 시작전 왼쪽으로 기울인다

################ Packet Structure
# Read or Write
read_Order = 0
write_Order = 1

# Device Number
wrist_Device = 3
elbow_Device = 2

# PID Number
exercise_Mode = 3
start_PID = 4
guidance_PID = 10
motor_angle = 12

#################################
global angle_wrist
global angle_elbow

angle_wrist = 0
angle_elbow = 0

global start_flag, quit_flag
start_flag = 0
quit_flag = False


######### construct_can_packet
def packet_construct(rw, part_Device, pid, data):
        Send = [0 for i in range(8)]
        
        Send[0]=rw
        Send[1]=part_Device
        Send[2]=(pid>>8)&0xFF
        Send[3]=pid&0xFF
        for i in range(0,4):
                Send[i+4]=struct.pack('f',data)[i]
        Send_packet = ('t1238'+str(bytes(Send).hex())+'\r').encode('utf-8')
        return Send_packet
    
######## parse_received_packet
def parsing_list(data):
        data_parse = list()
        if(data!=None):
                # Read/Write : 0 read 1 write
                data_parse.append(int(data.decode('utf-8')[5:7],16))

                # Target Device : 3 hand 2 wrist 2 elbow 1 shoulder
                data_parse.append(int(data.decode('utf-8')[7:9],16))

                # PID  : refer to datasheet (protocol)
                data_parse.append(int(data.decode('utf-8')[9:13],16))

                # Float data
                data_parse.append(round(struct.unpack('<f', bytes.fromhex(data.decode('utf-8')[13:21]))[0],3))
        else:
                data_parse = None

        return data_parse

########## quit
# quit_flag = None
def quit(ser):
    global quit_flag
    if keyboard.read_key() == "esc":
            quit_flag = True
            time.sleep(0.5)
            ser.write(packet_construct(write_Order, wrist_Device, start_PID, 0.0))
            time.sleep(0.5)
            ser.write(packet_construct(write_Order, elbow_Device, start_PID, 0.0))
            time.sleep(1)
            ser.close()
            
            print("Quit")
    
###########  data_receive         
def pos_rsv(ser):
        global angle_wrist
        global angle_elbow
        
        
        buffer = b''
        end_marker = b'\r'
        while True:
                read_data = ser.read_until(end_marker)
                if read_data:
                        buffer += read_data
                        if end_marker in buffer:
                                received_data = buffer[:buffer.index(end_marker)]
                                if(len(received_data)==21):
                                        pos_response = parsing_list(received_data)
                                        if(pos_response[1] == wrist_Device):
                                                if(pos_response[2]==motor_angle):
                                                        angle_wrist = pos_response[3]
                                        elif(pos_response[1] == elbow_Device):
                                                if(pos_response[2]==motor_angle):
                                                        angle_elbow = pos_response[3]

                                buffer = buffer[buffer.index(end_marker) + len(end_marker):]
        
############## data_write                                                                
def write_doc(ser):
    global angle_wrist
    global angle_elbow
    global start_flag, quit_flag
    
    file_index = 0  # 파일 고유 인덱스
    
    while not quit_flag:  # quit_flag가 True가 되면 루프 종료
        initime = datetime.now()
        # 파일 이름에 고유 인덱스를 추가하여 중복 방지
        file_name = r'C:\seoul_challenge\log' + '\\' + str(initime.strftime('%Y-%m-%d_%H-%M-%S')) + f'_log_{file_index}.csv'
        f = open(file_name, 'a', encoding='utf-8', newline='')
        wr = csv.writer(f, delimiter=',')
        title = ['Time', 'Wrist_Angle', 'Elbow_Angle']
        wr.writerow(title)

        buffer = []  # 데이터 일시 저장 버퍼
        while not quit_flag:  # quit_flag가 True가 되면 내부 루프 종료
            now = time.time()
            print(now)
            print('wrist:', angle_wrist)
            print('elbow:', angle_elbow)
            print(len(buffer))

            row = [str(now), str(angle_wrist), str(angle_elbow)]
            buffer.append(row)
            time.sleep(0.001)
            
            if len(buffer) >= 500:  # 버퍼가 500개 이상의 데이터가 쌓이면 파일에 저장
                break
        if buffer:
            wr.writerows(buffer)
            buffer.clear()
        f.close()
        file_index += 1  # 파일 인덱스 증가시켜 다음 파일 이름을 고유하게 함
    
    # 종료 시 남은 데이터 저장
    if buffer:
        # 마지막 남은 데이터를 새 파일에 저장 (필요 시 동일 파일에 저장하도록 로직 변경 가능)
        file_name = r'C:\seoul_challenge\log' + '\\' + str(initime.strftime('%Y-%m-%d_%H-%M-%S')) + f'_log_{file_index}.csv'
        f = open(file_name, 'a', encoding='utf-8', newline='')
        wr = csv.writer(f, delimiter=',')
        wr.writerows(buffer)
        f.close()

################ control and data receive process
# motor_angle = 0

def pos_call(ser):
        # 2: passive
        global start_flag
        if False:
            ser.write(packet_construct(write_Order, wrist_Device, guidance_PID, 1.0)) #100% = 1.0
            time.sleep(0.1)
            ser.write(packet_construct(write_Order, elbow_Device, guidance_PID, 1.0)) #100% = 1.0
            time.sleep(0.1)
            
            # ser.write(packet_construct(write_Order, wrist_Device, guidance_PID, 0.0)) #100% = 1.0
            # time.sleep(0.1)
            # ser.write(packet_construct(write_Order, elbow_Device, guidance_PID, 0.0)) #100% = 1.0
            # time.sleep(0.1)
            
            ser.write(packet_construct(write_Order, wrist_Device, exercise_Mode, 2.0))
            time.sleep(0.1)
            ser.write(packet_construct(write_Order, elbow_Device, exercise_Mode, 2.0))
            time.sleep(0.1)
            ser.write(packet_construct(write_Order, wrist_Device, start_PID, 1.0))
            time.sleep(0.1)
            ser.write(packet_construct(write_Order, elbow_Device, start_PID, 1.0))
            
        start_flag = 1
        time.sleep(0.05)
        while True:
                ser.write(packet_construct(read_Order, wrist_Device, motor_angle, 0))
                time.sleep(0.01)
                ser.write(packet_construct(read_Order, elbow_Device, motor_angle, 0))
                time.sleep(0.01)


############### main
### Serial Port Find
com_enable_flag = 0
try:
        #enter port Num
        port = 'COM7'
        
except:
        port = None
if(port==None):
        print("No Port Detected!")
else:
        print("Serial Port : ", port, "is detected and linked with device")
        ser = Serial(str(port),baudrate = 115200,timeout=0.001)
        com_enable_flag=1

        ### CSV making
        
        # enter the path
        # f=open('C://Users//choi//Desktop//data//240520data//'+str(initime.strftime('%Y-%m-%d_%H-%M-%S'))+'_log.csv', 'a', encoding = 'utf-8', newline='')


        ### Finish thread running
        quit_thread=threading.Thread(target=quit, args=(ser,))
        serial_thread=threading.Thread(target=pos_rsv, args=(ser,))
        write_thread=threading.Thread(target=write_doc, args=(ser,))
        pos_thread=threading.Thread(target=pos_call, args=(ser,))

        quit_thread.daemon = False
        serial_thread.daemon = True
        write_thread.daemon = True
        pos_thread.daemon = True

        quit_thread.start()
        serial_thread.start()
        write_thread.start()
        pos_thread.start()
