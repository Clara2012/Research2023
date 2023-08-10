from depthai_sdk import OakCamera, RecordType
import depthai as dai
from depthai_sdk.trigger_action import Trigger
from depthai_sdk.trigger_action.actions import RecordAction
from waiting import wait


with OakCamera() as oak:
    color = oak.create_camera('color', resolution='1080P', fps=20, encode='H265')
    left = oak.create_camera('left', resolution='800p', fps=20, encode='H265')
    right = oak.create_camera('right', resolution='800p', fps=20, encode='H265')

#this is manually starting the pipeline, whick is done in the oak.record or oak.create, but we must do it explisitly to enable frame synch mode
    pipeline = dai.Pipeline()

    RGB = pipeline.create(dai.node.ColorCamera)
    RGB.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)

    with dai.Device(pipeline) as device:
        queues = {}
        queues[RGB]= device.getOutputQueue(RGB)
        wait(lambda: queues.has() == True)

    # view trigger

    # Synchronize & save all (encoded) streams
    oak.record([color.out.encoded, left.out.encoded, right.out.encoded], './', RecordType.VIDEO)
    # Show color stream
    oak.visualize([color.out.camera], scale=2/3, fps=True)

    oak.start(blocking=True)
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

