import cv2
from threading import Event


class DroneLocator:
    # define folder
    def __init__(self, folder):
        self.folder = folder
        # connect to camera number 0. connects to web cam on my laptop
        self.video_capture = cv2.VideoCapture(0)

    # this could be done as a seperate thread/process
    def delete_the_weakest(self):
        pass

    def get_most_recent(self):
        ret, frame = self.video_capture.read()
        if not ret:
            # couldn't read frame, possibly kill drone
            pass
        return frame

    # todo: dakota and jen
    def locate_drone(self, pic, primary, secondary):
        pass

    def update(self, primary, secondary, stop):
        while not stop.is_set():
            picture = self.get_most_recent()
            self.locate_drone(picture, primary, secondary)
