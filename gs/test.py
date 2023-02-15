from socket import *
import signal
import cv2
import numpy as np
from io import BytesIO
from turbojpeg import TurboJPEG, TJPF_GRAY, TJSAMP_GRAY, TJFLAG_PROGRESSIVE, TJFLAG_FASTUPSAMPLE, TJFLAG_FASTDCT
import time
#s=socket(AF_INET, SOCK_DGRAM)
#s.bind(('',5600))
s=socket(AF_UNIX, SOCK_SEQPACKET)
s.connect("/home/ncer/testsocket")


data=b''
header=False

jpeg = TurboJPEG("/usr/lib/x86_64-linux-gnu/libturbojpeg.so")


while(True):
    temp=s.recv(65536)
    length=len(temp)
    print(length)
        
    try:

        img=jpeg.decode(temp)

        cv2.imshow("ss",img)

        cv2.waitKey(1)
    except Exception as e:
        print(e)
        pass

