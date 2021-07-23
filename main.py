import numpy as np
import tkinter as tk
import time

import controls


# 2x3 matrix to map 3D to 2D
base_projection = np.array([[10, 0, 0],
                            [0, 0, 10]], dtype=np.float64)


class Camera:
    def __init__(self):
        self.pos = np.array([10, 10, -10], dtype=np.float64).reshape((3, 1))
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
        # screen is a 3 x n matrix, each column representing coords of a value to be drawn

        # rotate world
        rotated_world = np.dot(self.create_rotation_matrix(), world)
        screen = np.dot(base_projection, self.pos - rotated_world)

        return screen


camera = Camera()


class Renderer:
    def __init__(self, tkinter_canvas, camera):
        self._canvas: tk.Canvas = tkinter_canvas
        self._camera: Camera = camera

    def clear(self):
        self._canvas.delete("all")

    def render_origin_marker(self):
        origin = [0, 0, 0]
        self.render_line(origin, [5, 0, 0], arrow="last", fill="red")
        self.render_line(origin, [0, 5, 0], arrow="last", fill="green")
        self.render_line(origin, [0, 0, 5], arrow="last", fill="blue")
        self.render_point(origin, 3)


    def render_point(self, a, radius):
        """
            Projects and renders a point onto the canvas, surrounded by a circle
            
            a - array (length 3) representing a 3d point

            radius - an integer representing the radius of the 2d circle
        """
        point = np.array([a]).T
        a2d = self._camera.project(point).T[0]
        self._canvas.create_oval(*(a2d - radius), *(a2d + radius), fill="red")


    def render_line(self, a, b, **kwargs):
        """
            Projects and renders a line onto the canvas

            a, b - arrays (length 3) representing two 3d points

            **kwargs - extra kwargs for the tkinter create_line function
        """
        points = np.array([a, b]).T
        projected = self._camera.project(points)
        a2d, b2d = projected.T
        self._canvas.create_line(a2d[0], a2d[1], b2d[0], b2d[1], **kwargs)

    def render_quad(self, a, b, c, d):
        """
            Projects and renders a line onto the canvas

            a, b, c, d - arrays (length 3) representing four 3d points
        """
        points = np.array([a, b, c, d]).T
        projected = self._camera.project(points)
        a2d, b2d, c2d, d2d = projected.T
        self._canvas.create_polygon(
            *a2d, *b2d, *c2d, *d2d, width=3, outline="black", fill="")

    def render_cube(self, a, b, fill=None):
        """
            Projects and renders a line onto the canvas

            a, b - arrays (length 3) representing two 3d points on opposite corners of the cube
            fill - colour for the faces to be filled (optional)
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
        projected = self._camera.project(points)
        p, q, r, s, t, u, v, w = projected.T
        
        # FACES
        if fill != None:
            self._canvas.create_polygon(*p, *q, *r, *s, fill=fill, outline="")
            self._canvas.create_polygon(*t, *u, *v, *w, fill=fill, outline="")
            self._canvas.create_polygon(*p, *q, *u, *t, fill=fill, outline="")
            self._canvas.create_polygon(*r, *s, *w, *v, fill=fill, outline="")
            self._canvas.create_polygon(*p, *s, *w, *t, fill=fill, outline="")
            self._canvas.create_polygon(*q, *r, *v, *u, fill=fill, outline="")
        
        # EDGES
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

def update_canvas(renderer: Renderer):

    renderer.clear()

    p1 = [2, 2, 5]
    p2 = [2, 30, 5]
    p3 = [30, 30, 5]
    p4 = [30, 2, 5]
    renderer.render_quad(p1, p2, p3, p4)
    renderer.render_cube([2, 2, 2], [9, 9, 9], fill="red")

    # draw origin
    # renderer.render_point([0, 0, 0], 5)
    renderer.render_origin_marker()


render_batch_start_time = None
RENDER_BATCH_SIZE = 60
TARGET_FPS = 40


def frame(n=0):
    global camera, render_batch_start_time
    # camera.yaw += 0.005

    # measure performance by measuring frame rate in batches of 20
    if n % RENDER_BATCH_SIZE == 0:
        if render_batch_start_time is not None:
            batch_duration_ms = 1000 * \
                (time.perf_counter() - render_batch_start_time)
            print(f"Render x {RENDER_BATCH_SIZE} took {batch_duration_ms:4.0f} ms ({batch_duration_ms / RENDER_BATCH_SIZE:3.0f} ms avg, {1000 * RENDER_BATCH_SIZE / batch_duration_ms:3.0f} FPS)")
        render_batch_start_time = time.perf_counter()
    update_canvas(renderer)

    window.after(1000 // TARGET_FPS, lambda: frame(n + 1))


window = tk.Tk()
window.title("Projection Matrices")
canvas = tk.Canvas(width=600, height=450, bg="white")
renderer = Renderer(canvas, camera)
canvas.pack()
update_canvas(renderer)
rotation_drag_handler = controls.RotationDragHandler(camera)
rotation_drag_handler.bind_event_handlers(window, 1)
position_drag_handler = controls.PositionDragHandler(camera)
position_drag_handler.bind_event_handlers(window, 3)
window.after(1000 // TARGET_FPS, frame)
window.mainloop()
