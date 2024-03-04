# -*- coding: utf-8 -*-
"""
Created on Wed Nov  1 18:31:23 2023

@author: user
"""
import socket
import struct
import numpy as np
socket.setdefaulttimeout(5)


class TCPclient:
    def __init__(self, ip, port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port))

    def sendmsg(self, msg):
        self.client.send(struct.pack('d', msg))

    def receivemsg(self):  # just receive a set
        # check simulink send how many variables to python
        # in this case: setting 48 means sending 6 vars (each var occupy 8 bits)
        # 6 vars: dis, spd, af, Vx, ap, ACC_Status
        data = self.client.recv(48)  # dis, spd, af, vf, ap,ACC_Status

        if len(data) == 0:
            print('server closed')
            return
        elif len(data) != 48:
            print(f'len not right: {len(data)}')
            return

        dis = np.asarray(struct.unpack('d', data[8*0:8*(0+1)]))
        relative_speed = np.asarray(struct.unpack('d', data[8*1:8*(1+1)]))
        af = np.asarray(struct.unpack('d', data[8*2:8*(2+1)]))
        vf = np.asarray(struct.unpack('d', data[8*3:8*(3+1)]))
        ap = np.asarray(struct.unpack('d', data[8*4:8*(4+1)]))
        ACC_status = np.asarray(struct.unpack('d', data[8*5:8*(5+1)]))

        return dis, relative_speed, af/10, vf, ap/10, ACC_status
