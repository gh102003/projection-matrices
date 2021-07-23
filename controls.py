from abc import ABC, abstractmethod  # abstract base class
import numpy as np


class DragHandler(ABC):

    def __init__(self, camera):
        self._camera = camera
        self.__last_x = None
        self.__last_y = None

    def bind_event_handlers(self, tk_widget, button_number):
        tk_widget.bind("<B" + str(button_number) + "-Motion>", self.drag)
        tk_widget.bind("<ButtonRelease-" +
                       str(button_number) + ">", self.release)

    # called directly by tkinter event handler
    def drag(self, event):
        if self.__last_x is not None:
            delta_x = event.x - self.__last_x
            self.change_x(delta_x)

        if self.__last_y is not None:
            delta_y = event.y - self.__last_y
            self.change_y(delta_y)

        self.__last_x = event.x
        self.__last_y = event.y

    # called directly by tkinter event handler
    def release(self, event):
        self.__last_x = None
        self.__last_y = None

    @abstractmethod
    def change_x(self, delta_x):
        pass

    @abstractmethod
    def change_y(self, delta_y):
        pass


class RotationDragHandler(DragHandler):
    def change_x(self, delta_x):
        self._camera.yaw += delta_x / 200

    def change_y(self, delta_y):
        self._camera.pitch += delta_y / 200


class PositionDragHandler(DragHandler):
    def change_x(self, delta_x):
        self._camera.pos[0, 0] += delta_x / 10

    def change_y(self, delta_y):
        self._camera.pos[2, 0] += delta_y / 10
