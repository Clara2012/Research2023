from operator import index
import subprocess
import os
import datetime
import cv2
import depthai as dai
import contextlib
import shutil
import subprocess
import sys
import signal
from tkinter import *
import time
from threading import *
import datetime

# create object
root = Tk()

# set geometry
root.geometry("500x200")
root.title("Recording GUI")
root.iconbitmap(r"C:\Users\clevy\OAK-D-Recordings\python-recorder-main\camera_icon.ico")

# setting the number of columns
root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=1)
root.columnconfigure(2, weight=2)
root.columnconfigure(3, weight=1)
root.columnconfigure(4, weight=1)


# setting the number of rows
root.rowconfigure(0, weight=1)
root.rowconfigure(1, weight=1)
root.rowconfigure(2, weight=1)


#recording happens here
with contextlib.ExitStack() as stack:
    deviceInfos = dai.Device.getAllAvailableDevices()
    #usbSpeed = dai.UsbSpeed.SUPER
    openVinoVersion = dai.OpenVINO.Version.VERSION_2021_4

    #qRgbMap = []
    devices = []



#print number of connected devices
for deviceInfo in deviceInfos:
        deviceInfo: dai.DeviceInfo
        device: dai.Device = stack.enter_context(dai.Device(openVinoVersion, deviceInfo))
        devices.append(device)
        print("=== Connected to ", deviceInfo.getMxId())
        


def start_record():
    # remove the h265/h264 files saved if they exist
 
    if os.path.exists("color",deviceInfos,".h265"):
      os.remove("color",deviceInfos,".h265")
    if os.path.exists("mono1",device.index(deviceInfo),".h264"):
      os.remove("mono1",device.index(deviceInfo),".h264")
    if os.path.exists("mono2",device.index(deviceInfo),".h264"):
      os.remove("mono2",device.index(deviceInfo),".h264")
   
     # will run this command to open the encoding.py file
    global process
    cmd = "py encoding.py"
    process_label['text']= "Recording Started..."
    start_btn['bg'] ="red"
    process = subprocess.Popen(args=cmd)
    # change the process text        

# use threading
def threading():
    # call the work function
    global t1
    t1 = Thread(target=start_record)
    t1.start()


def stop_record():
    process_label['text'] = "Recording Stopped..."
    # setting start button back to default colour
    start_btn['bg'] = orig_colour
    subprocess.Popen.terminate(process)

# move videos to destination folder
def save_video():
    # get date
    now = str(datetime.date.today())

    # convert the video files to mp4
    os.system("cmd /c ffmpeg -framerate 25 -i mono1.h264 -c copy mono1.mp4")
    os.system("cmd /c ffmpeg -framerate 25 -i mono2.h264 -c copy mono2.mp4")
    os.system("cmd /c ffmpeg -framerate 25 -i color.h265 -c copy color.mp4")

    # variables to store files name + location before they are moved
    old_name_color = r"C:\Users\clevy\OAK-D-Recordings\python-recorder-main\color.mp4"
    old_name_mono1 = r"C:\Users\clevy\OAK-D-Recordings\python-recorder-main\mono1.mp4"
    old_name_mono2 = r"C:\Users\clevy\OAK-D-Recordings\python-recorder-main\mono2.mp4"

    name_color =  now+"_color.mp4"
    name_mono1 = now + "_mono1.mp4"
    name_mono2 = now + "_mono2.mp4"

    file_name = entry.get()



    # variables to store files name + location destination
    new_name_color = "C:\\Camera_Recordings\\" + name_color + file_name 
    new_name_mono1 = "C:\\Camera_Recordings\\" + name_mono1 + file_name
    new_name_mono2 = "C:\\Camera_Recordings\\" + name_mono2 + file_name

    # rename file + move it
    os.rename(old_name_color, new_name_color)
    os.rename(old_name_mono1, new_name_mono1)
    os.rename(old_name_mono2, new_name_mono2)

#all display box stuff below
# create label
label = Label(root, text="SaMS Lab: Oak-D Lite Recording GUI", font='bold')
label.grid(row=0, column=2, sticky="N")
process_label = Label(root, text='Click "Start Recording" to begin')
process_label.grid(row=1, column=2, sticky="N")

# creating text input field
entry = Entry(root)
entry.grid(row=1, column=2, sticky="S")
entry.grid_forget()

# create button
start_btn = Button(root, text="Start Recording", command=threading, width=15)
start_btn.grid(row=2, column=1, pady=2)

stop_btn = Button(root, text= "Stop Recording", command=stop_record, width=15)
stop_btn.grid(row=2, column=2, pady=2)

save_btn = Button(root, text="Save Video", command=save_video, width=15)
# getting the original button colour
orig_colour = save_btn.cget("bg")
save_btn.grid(row=2, column=3)

# execute tkinter
root.mainloop()

# notes
# camera temperature over 105C, automatically shuts down 7mins 45 seconds








