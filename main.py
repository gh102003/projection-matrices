from PIL import Image, ImageTk
import numpy as np
import itertools
import tkinter as tk
import time


# 2x3 matrix to map 3D to 2D
base_projection = np.array([[10, 0, 0],
                            [0, 0, 10]], dtype=np.float64)


class Camera:
    def __init__(self):
        self.pos = np.array([0, 0, 0], dtype=np.float64).reshape((3, 1))
        # pitch, roll, and yaw are angles (in radians) of anticlockwise rotations about the x, y and z axes respectively
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

    def create_rotation_matrix(self):
        rotationX = np.array([[1, 0, 0],
                              [0, np.cos(self.pitch), -np.sin(self.pitch)],
                              [0, np.sin(self.pitch), np.cos(self.pitch)]])

        rotationY = np.array([[np.cos(self.roll), 0, np.sin(self.roll)],
                              [0, 1, 0],
                              [-np.sin(self.roll), 0, np.cos(self.roll)]])

        rotationZ = np.array([[np.cos(self.yaw), -np.sin(self.yaw), 0],
                              [np.sin(self.yaw), np.cos(self.yaw), 0],
                              [0, 0, 1]])

        rotation = np.dot(rotationX, np.dot(rotationY, rotationZ))

        return rotation

    def project(self, world):
        """
        Takes an array of 3d points and projects them into an array of 2d points
        world - 3xn matrix, each column representing a 3d point
        returns a 2xn matrix, each column representing a 2d point
        """
        # start = time.perf_counter()
        # screen is a 3 x n matrix, each column representing coords of a value to be drawn
        screen = np.dot(base_projection, self.pos -
                        np.dot(self.create_rotation_matrix(), world))
        # print(
        # f"Projection took {1000 * (time.perf_counter() - start):4.0f} ms")
        return screen


camera = Camera()


class Renderer:
    def __init__(self, tkinter_canvas):
        self._canvas: tk.Canvas = tkinter_canvas

    def render_line(self, a, b):
        """
            Projects and renders a line onto the canvas

            a, b - arrays (length 3) representing two 3d points
        """
        points = np.array([a, b]).T
        projected = camera.project(points)
        a2d, b2d = projected.T
        self._canvas.create_line(a2d[0], a2d[1], b2d[0], b2d[1])

    def render_quad(self, a, b, c, d):
        """
            Projects and renders a line onto the canvas

            a, b, c, d - arrays (length 3) representing four 3d points
        """
        points = np.array([a, b, c, d]).T
        projected = camera.project(points)
        a2d, b2d, c2d, d2d = projected.T
        self._canvas.create_polygon(
            *a2d, *b2d, *c2d, *d2d, width=3, outline="black", fill="")

    def render_cube(self, a, b):
        """
            Projects and renders a line onto the canvas

            a, b - arrays (length 3) representing two 3d points on opposite corners of the cube
        """
        points = np.array([
            # bottom
            a,
            [b[0], a[1], a[2]],
            [b[0], b[1], a[2]],
            [a[0], b[1], a[2]],
            # top
            [a[0], a[1], b[2]],
            [b[0], a[1], b[2]],
            b,
            [a[0], b[1], b[2]],
        ]).T
        projected = camera.project(points)
        p, q, r, s, t, u, v, w = projected.T
        # bottom
        self._canvas.create_line(*p, *q)
        self._canvas.create_line(*q, *r)
        self._canvas.create_line(*r, *s)
        self._canvas.create_line(*s, *p)
        # top
        self._canvas.create_line(*t, *u)
        self._canvas.create_line(*u, *v)
        self._canvas.create_line(*v, *w)
        self._canvas.create_line(*w, *t)
        # vertical
        self._canvas.create_line(*p, *t)
        self._canvas.create_line(*q, *u)
        self._canvas.create_line(*r, *v)
        self._canvas.create_line(*s, *w)


# RENDERING ONTO CANVAS

def render_PIL(canvas, renderer):
    
    canvas.delete("all")

    p1 = [2, 2, 5]
    p2 = [2, 30, 5]
    p3 = [30, 30, 5]
    p4 = [30, 2, 5]
    renderer.render_quad(p1, p2, p3, p4)
    renderer.render_cube([2, 2, 2], [9, 9, 9])

    # draw origin
    origin = camera.project(np.array([[0, 0, 0]]).T).T[0]
    canvas.create_oval(origin[0], origin[1],
                       origin[0] + 8, origin[1] + 8, fill="red")


def key_press(event):
    global camera

    if event.char == "w":
        camera.pos[2, 0] += 1
    if event.char == "s":
        camera.pos[2, 0] += -1
    if event.char == "a":
        camera.pos[0, 0] += 1
    if event.char == "d":
        camera.pos[0, 0] += -1
    if event.char == "q":
        camera.roll += -0.05
    if event.char == "e":
        camera.roll += 0.05
    if event.char == "r":
        camera.pitch += -0.05
    if event.char == "f":
        camera.pitch += 0.05

    # render_PIL(canvas, renderer)

render_batch_start_time = None
render_batch_size = 20
def frame(n=0):
    global camera, render_batch_start_time
    camera.yaw += 0.005
    
    # measure performance by measuring frame rate in batches of 20
    if n % render_batch_size == 0:
        if render_batch_start_time is not None:
            batch_duration_ms = 1000 * (time.perf_counter() - render_batch_start_time)
            print(f"Render x {render_batch_size} took {batch_duration_ms:4.0f} ms ({batch_duration_ms / render_batch_size:2.0f} ms avg, {batch_duration_ms / render_batch_size:2.0f} FPS)")
        render_batch_start_time = time.perf_counter()
    render_PIL(canvas, renderer)


    window.after(30, lambda: frame(n + 1))



last_b1_x = None
last_b1_y = None

def b1_drag(event):
    global camera, last_b1_x, last_b1_y

    if last_b1_x is not None:
        delta_x = event.x - last_b1_x
        camera.yaw += delta_x / 100

    if last_b1_y is not None:
        delta_y = event.y - last_b1_y
        camera.pitch += delta_y / 100

    last_b1_x = event.x
    last_b1_y = event.y

def b1_release(event):
    global last_b1_x, last_b1_y
    last_b1_x = None
    last_b1_y = None


last_b2_x = None
last_b2_y = None

def b3_drag(event):
    global camera, last_b2_x, last_b2_y

    if last_b2_x is not None:
        delta_x = event.x - last_b2_x
        camera.pos[0, 0] += delta_x / 10

    if last_b2_y is not None:
        delta_y = event.y - last_b2_y
        camera.pos[2, 0] += delta_y / 10

    last_b2_x = event.x
    last_b2_y = event.y

def b3_release(event):
    global last_b2_x, last_b2_y
    last_b2_x = None
    last_b2_y = None


window = tk.Tk()
window.title("Projection Matrices")
canvas = tk.Canvas(width=400, height=400, bg="white")
renderer = Renderer(canvas)
canvas.pack()
render_PIL(canvas, renderer)
window.bind("<B1-Motion>", b1_drag)
window.bind("<ButtonRelease-1>", b1_release)
window.bind("<B3-Motion>", b3_drag)
window.bind("<ButtonRelease-3>", b3_release)
# window.bind("w", key_press)
# window.bind("s", key_press)
# window.bind("a", key_press)
# window.bind("d", key_press)
# window.bind("q", key_press)
# window.bind("e", key_press)
# window.bind("r", key_press)
# window.bind("f", key_press)
window.after(30, frame)
window.mainloop()
