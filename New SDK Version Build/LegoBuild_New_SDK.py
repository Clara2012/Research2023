from depthai_sdk import OakCamera, RecordType
import depthai as dai
from depthai_sdk.trigger_action import Trigger
from depthai_sdk.trigger_action.actions import RecordAction

with OakCamera() as oak:
    color = oak.create_camera('color', resolution='1080P', fps=20, encode='H265')
    left = oak.create_camera('left', resolution='800p', fps=20, encode='H265')
    right = oak.create_camera('right', resolution='800p', fps=20, encode='H265')

#this is manually starting the pipeline, whick is done in the oak.record or oak.create, but we must do it explisitly to enable frame synch mode
    pipeline = dai.Pipeline()

    RGB = pipeline.create(dai.node.ColorCamera)

    RGB.initialControl.setFrameSyncMode(dai.CameraControl.FrameSyncMode.INPUT)

    # Synchronize & save all (encoded) streams
    oak.record([color.out.encoded, left.out.encoded, right.out.encoded], './', RecordType.VIDEO)
    # Show color stream
    oak.visualize([color.out.camera], scale=2/3, fps=True)

    oak.start(blocking=True)


    #TODO:
    # use trigger to synch streams
    # record with mjpeg, must use av
    # record at higher FPS
    # record with two cameras
    # read FSYNCH trigger from cameras

