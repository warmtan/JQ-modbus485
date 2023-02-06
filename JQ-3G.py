import serial
import numpy
import time as sleeep
from binascii import *
import crcmod

ser = serial.Serial()
 
def port_open():
    ser.port = "/dev/ttyUSB0" #设置端口号
    ser.baudrate = 115200     #设置波特率
    ser.bytesize = 8          #设置数据位
    ser.stopbits = 1          #设置停止位
    ser.parity = "N"          #设置校验位
    ser.open()              #打开串口,要找到对的串口号才会成功
    if(ser.isOpen()):
        print("打开成功")
    else:
        print("打开失败")
 
def port_close():
    ser.close()
    if (ser.isOpen()):
        print("关闭失败")
    else:
        print("关闭成功")
 
def send(send_data):
    if (ser.isOpen()):
        ser.write(send_data)  #Hex发送
        print("发送成功",send_data)
    else:
        print("发送失败")
def crc16Add(read):  
    crc16 = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
    data = read.replace(" ", "") #消除空格
    readcrcout = hex(crc16(unhexlify(data))).upper()
    str_list = list(readcrcout)
    # print(str_list)
    if len(str_list) == 5:
        str_list.insert(2, '0')  # 位数不足补0，因为一般最少是5个
    crc_data = "".join(str_list) #用""把数组的每一位结合起来  组成新的字符串
    # print(crc_data)
    read = read.strip() + ' ' + crc_data[4:] + ' ' + crc_data[2:4] #把源代码和crc校验码连接起来
    # print('CRC16校验:', crc_data[4:] + ' ' + crc_data[2:4])
    # print(read)
    return read
#手掌开关数据帧 03EB 位置 速度 力
def OpenHand(hand):
    for i in range(len(hand)):
        hand[i] = round(2.55*hand[i]) # 转为整数
        hand[i] = hex(hand[i]) #
        hand[i] = hand[i][2:]
        hand[i] = hand[i].upper()
        i+=1 
    if int(hand[0], 16) < 10:
        hand[0] = "0"+hand[0]
    print(hand[0])
    hand = hand[0]+ " " + hand[1]+ " " +hand[2] + " " + "00"
    str ="09 10 03 EB 00 02 04 "
    hand = str + hand
    hand = crc16Add(hand)
    hand = hand.split(' ')
    for i in range(len(hand)):
        hand[i] = '0x' + hand[i]
        hand[i] =hand[i].replace(" ", "")
        hand[i] =hand[i].replace(",", "")
        hand[i] = int(hand[i], 16)
        # hand[i] = hex(hand[i])
        i+=1
    return hand
#手掌旋转数据帧 03ED
def  RevoleHand(revolve):
    for i in range(len(revolve)):
        revolve[i] = round(2.55*revolve[i]) # 转为整数
        revolve[i] = hex(revolve[i]) #
        revolve[i] = revolve[i][2:]
        revolve[i] = revolve[i].upper()
        i+=1 
    if int(hand[0], 16) < 10:
        hand[0] = "0"+hand[0]
    print(hand[0])
    revolve = revolve[0]+ " " + revolve[1]+ " " +revolve[2] + " " + "00"
    str ="09 10 03 ED 00 02 04 "
    revolve = str + revolve
    revolve = crc16Add(revolve)
    revolve = revolve.split(' ')
    for i in range(len(revolve)):
        revolve[i] = '0x' + revolve[i]
        revolve[i] =revolve[i].replace(" ", "")
        revolve[i] =revolve[i].replace(",", "")
        revolve[i] = int(revolve[i], 16)
        # hand[i] = hex(hand[i])
        i+=1
    return revolve
# #左指开合数据帧 03EF
def LeftHand(left):
    for i in range(len(left)):
        left[i] = round(2.55*left[i]) # 转为整数
        left[i] = hex(left[i]) #
        left[i] = left[i][2:]
        left[i] = left[i].upper()
        i+=1 
    if int(hand[0], 16) < 10:
        hand[0] = "0"+hand[0]
    print(hand[0])
    left = left[0]+ " " + left[1]+ " " +left[2] + " " + "00"
    str ="09 10 03 EF 00 02 04 "
    left = str + left
    left = crc16Add(left)
    left = left.split(' ')
    for i in range(len(revolve)):
        left[i] = '0x' + left[i]
        left[i] =left[i].replace(" ", "")
        left[i] =left[i].replace(",", "")
        left[i] = int(left[i], 16)
        # hand[i] = hex(hand[i])
        i+=1
    return left
# #右手开合数据帧 03F3
def RightHand(right):
    for i in range(len(right)):
        right[i] = round(2.55*right[i]) # 转为整数
        right[i] = hex(right[i]) #
        right[i] = right[i][2:]
        right[i] = right[i].upper()
        i+=1
    if int(hand[0], 16) < 10:
        hand[0] = "0"+hand[0]
    print(hand[0])
    right = right[0]+ " " + right[1]+ " " +right[2] + " " + "00"
    str ="09 10 03 F3 00 02 04 "
    right = str + right
    right = crc16Add(right)
    right = right.split(' ')
    for i in range(len(right)):
        right[i] = '0x' + right[i]
        right[i] =right[i].replace(" ", "")
        right[i] =right[i].replace(",", "")
        right[i] = int(right[i], 16)
        # hand[i] = hex(hand[i])
        i+=1
    return right
# #中指开合数据帧 03F1
def CenterHand(center):
    for i in range(len(center)):
        center[i] = round(2.55*center[i]) # 转为整数
        center[i] = hex(center[i]) #
        center[i] = center[i][2:]
        center[i] = center[i].upper()
        i+=1
    if int(hand[0], 16) < 10:
        hand[0] = "0"+hand[0]
    print(hand[0]) 
    center = center[0]+ " " + center[1]+ " " +center[2] + " " + "00"
    str ="09 10 03 F1 00 02 04 "
    center = str + center
    center = crc16Add(center)
    center = center.split(' ')
    for i in range(len(revolve)):
        center[i] = '0x' + center[i]
        center[i] =center[i].replace(" ", "")
        center[i] =center[i].replace(",", "")
        center[i] = int(center[i], 16)
        # hand[i] = hex(hand[i])
        i+=1
    return center
def jq_init():
    # activation 09 06 03 E8 00 FF 48 B2
    act = [0x09, 0x06, 0x03, 0xE8, 0x00, 0xFF,0x48, 0xB2]
    send(act)
    # sleeep(10)
    # # Read 09 03 07 D0 00 01 85 CF
    # rea = [0x09, 0x03, 0x07, 0xD0, 0x00, 0x01,0x85, 0xCF]
    # send(rea) 
if __name__ == "__main__":
    port_open()
    # jq_init()
    # close-grasp
    # data = [0x09,0x10,0x03,0xEB,0x00,0x02,0x04,0xFF,0xFF,0x6E,0x00,0xAE,0x80]
    # print(type(data[3]))
    # # open-grasp
    # data = [0x09,0x10,0x03,0xEB,0x00,0x02,0x04,0x00,0xFF,0x6E,0x00,0x9E,0x94]
    # close hand
    # data = [0x09,0x10,0x03,0xED,0x00,0x02,0x04,0x00,0x1F,0x6E,0x00,0x1F,0x48]
    # open hand
    # data =  [0x09,0x10,0x03,0xED,0x00,0x02,0x04,0xFF,0x1F,0x6E,0x00,0x2F,0x5C]
    # open hand
    # data =  [5,5,5]
    # data =  OpenHand(data)
    # print(data)
    hand =[40,50,43]
    hand =RightHand(hand)
    data = hand
    # revolve=[50,50,43]
    # revolve=RevoleHand(revolve)
    # # print(hand)
    # data = revolve
    send(data)