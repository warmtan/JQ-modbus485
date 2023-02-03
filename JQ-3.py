import serial
import numpy
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
#生成CRC16-MODBUS校验码
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
    return read
#手掌开关数据帧 03EB
def DataHand(hand):
    data3Hand[6] = [0x09,0x10,0x03,0xEB,0x00,0x02,0x04,0x00,0x00,0x00,0x00]
    return hand
#手掌旋转数据帧 03ED
def OpenHand(open):
    data3Hand[6] = [0x09,0x10,0x03,0xED,0x00,0x02,0x04,0x00,0x00,0x00,0x00]
    return open
#左指开合数据帧 03EF
def LeftHand(left):
    Leftdata[6] = [0x09,0x10,0x03,0xEF,0x00,0x02,0x04,0x00,0x00,0x00,0x00]
    return left
#右手开合数据帧 03F3
def RightHand(right):
    Leftdata[6] = [0x09,0x10,0x03,0xF3,0x00,0x02,0x04,0x00,0x00,0x00,0x00]
    return right
#中指开合数据帧 03F1
def CenterHand(center):
    Leftdata[6] = [0x09,0x10,0x03,0xF1,0x00,0x02,0x04,0x00,0x00,0x00,0x00]
    return center
# 示例:全力关闭夹爪 09 10 03 EB 00 02 04 FF FF 6E 00 AE 80
# 09 10 寄存器地址[03 ] 0002 写入的寄存器数量 04数据字节数 FFFF 寄存器 03EB 的内容(位置：FF,速度：FF)
# 6E00 寄存器 03EC 的内容(6E:电压)
# AE 80
if __name__ == "__main__":
    # port_open()
    data = crc16Add("09 10 03 EB 00 02 04 F0 FF 6E 00")
    data = data.split(' ')
    for i in range(len(data)):
        data[i] = '0x' + data[i]
    for i in range(len(data)):
        data[i]=hex(ord(data[i]))
    print(data)
    # int(str, n) 如 int("0x2d",16), int("2d",16) n进制的字符串转换成十进制整数
    for i in range(len(data)):
        data[i]=int(data[i],16)
    for i in range(len(data)):
        data[i]=hex(data[i])
    print(data)
    # print(hex(data[0]))
    # send(data)