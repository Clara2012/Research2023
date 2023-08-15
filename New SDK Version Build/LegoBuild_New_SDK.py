from depthai_sdk import OakCamera, RecordType
import depthai as dai
from depthai_sdk.trigger_action import Trigger
from depthai_sdk.trigger_action.actions import RecordAction
from waiting import wait
import time



with OakCamera() as oak:
    color = oak.create_camera('color', resolution='1080P', fps=20, encode='H265')
    left = oak.create_camera('left', resolution='800p', fps=20, encode='H265')
    right = oak.create_camera('right', resolution='800p', fps=20, encode='H265')

    #enable reading the trigger
    color.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
    left.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)
    right.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)

    # create pipeline so we can write explicitly to it:
    pipeline = oak.build()

#this is manually starting the pipeline, whick is done in the oak.record or oak.create, but we must do it explisitly to enable frame synch mode

    # script to read if trigger is sent
    """""
    def trig():
        device = dai.Device(pipeline)
        queues = {}
        oakQueue = device.getOutputQueue()
        wait(lambda: oakQueue.has() == True)
        return (time.time)
        #return time or bool to synch with
 """""
    # Synchronize & save all (encoded) streams
    oak.record([color.out.encoded, left.out.encoded, right.out.encoded], './', RecordType.VIDEO)

    # Show color stream
    oak.visualize([color.out.camera], scale=2/3, fps=True)

    # trigger = DetectionTrigger(input=Fsynch input, sent = True,)

    # oak.trigger_action([color.out.camera], trig, oak.start(blocking=True))

    # oak.start uploads the pipeline to the device
    # I cannot access them to see if queues name has for trigger
    # This may be a new node ive added??? Try checking that out
    print()

    #TODO:
    # use trigger to synch streams
    # record with mjpeg, must use av
    # record at higher FPS
    # record with two cameras
    # read FSYNCH trigger from cameras

