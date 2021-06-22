from __future__ import print_function
import matplotlib.pyplot as plt
import numpy as np
import serial
import time
import io, datetime
data = []
# XBee setting
serdev1 = '/dev/ttyUSB0'
s = serial.Serial(serdev1, 9600)
filename="data_file.txt"
f   = open("data_file.txt", "a", encoding='UTF-8')


s.write("+++".encode())
char = s.read(2)
print("Enter AT mode.")
print(char.decode())

s.write("ATMY 0x164\r\n".encode())
char = s.read(3)
print("Set MY 0x164.")
print(char.decode())

s.write("ATDL 0x264\r\n".encode())
char = s.read(3)
print("Set DL 0x264.")
print(char.decode())

s.write("ATID 0x1\r\n".encode())
char = s.read(3)
print("Set PAN ID 0x1.")
print(char.decode())

s.write("ATWR\r\n".encode())
char = s.read(3)
print("Write config.")
print(char.decode())

s.write("ATMY\r\n".encode())
char = s.read(4)
print("MY :")
print(char.decode())

s.write("ATDL\r\n".encode())
char = s.read(4)
print("DL : ")
print(char.decode())

s.write("ATCN\r\n".encode())
char = s.read(3)
print("Exit AT mode.")
print(char.decode())

print("start sending RPC")
while True:
    s = s.readline()
    line = s.decode('utf-8').replace(",", "\r\n")
    time.sleep(.1)
    f.write(line+"\r\n")    # Appends output to file
s.close()
