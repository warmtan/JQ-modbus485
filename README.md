第一步连接
<!--  -->
    demo1.py 实现
    ser.port = "/dev/ttyUSB0" #设置端口号
    ser.baudrate = 115200     #设置波特率
    ser.bytesize = 8          #设置数据位
    ser.stopbits = 1          #设置停止位
    ser.parity = "N"          #设置校验位
<!--  -->


第二步CRC教验码(通用的)
    demo2.py 实现

    close-grasp
    09 10 03 EB 00 02 04 FF FF 6E 00
    09 10 03 EB 00 02 04 FF FF 6E 00 AE 80
    
    open-grasp
    09 10 03 EB 00 02 04 00 FF 6E 00
    09 10 03 EB 00 02 04 00 FF 6E 00 9E 94