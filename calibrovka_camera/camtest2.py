from picamera2 import Picamera2, Preview
import time

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(1)

while True:
    frame = picam2.capture_array()
    print("FRAME", frame.shape)

picam2.stop()
