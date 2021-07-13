from PIL import Image, ImageTk
import numpy as np
import itertools
import tkinter as tk
import time

# radius = 10, centre = (0, 0, 0)
def point_in_sphere(point):
    if point.shape != (3,):
        raise ValueError

    orig_dist = (point ** 2).sum() ** 0.5
    return orig_dist <= 10


# sidelength = 20, centre = (0, 0, 0)
def point_in_cube(point):
    if point.shape != (3,):
        raise ValueError

    # true if all ordinates between -10 and 10
    return np.all([-10 <= point, point <= 10])


# world is a 3 x n matrix, each column representing a possible coord
axis_values = np.linspace(-10, 10, 80)
# world = np.array(list(itertools.product(
    # axis_values, axis_values, axis_values)))
# world = list(filter(point_in_cube, world))
# world = np.array(world).T
world = np.array(np.meshgrid(axis_values, axis_values, axis_values)).T.reshape(-1, 3).T


# 2x3 matrix to map 3D to 2D
base_projection = np.array([[10, 0, 0],
                            [0, 0, 10]])


class Camera:
    def __init__(self):
        self.pos = np.array([0, 0, 0]).reshape((3, 1))
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
        start = time.perf_counter()
        # screen is a 3 x n matrix, each column representing coords of a value to be drawn
        screen = np.dot(base_projection, np.dot(
            self.create_rotation_matrix(), world - self.pos))
        print(
            f"Projection took {1000 * (time.perf_counter() - start):4.0f} ms")
        return screen


camera = Camera()

# RENDERING ONTO CANVAS

def render_PIL(canvas):
    start = time.perf_counter()
    canvas.delete("all")

    # draw points
    # points = camera.project(world).T
    points = camera.project(world)
    pixels = np.full((400, 400), 255, dtype=np.uint8)
    a = time.perf_counter()
    converted = (points).astype(int, casting='unsafe') + 200 # convert to ints and move origin to screen centre

    # filter out any coords with a value below 0 (would cause index to wrap around)
    filtered = converted[:, np.all(
        [converted[0] >= 0, converted[1] >= 0, converted[0] < 400, converted[1] < 400], axis=0)]

    try:
        pixels[filtered[1], filtered[0]] = 0
    except IndexError:
        pass

    print(f"Points -> Pixels took {1000 * (time.perf_counter() - a):4.0f} ms")

    window.image = ImageTk.PhotoImage(Image.fromarray(
        pixels, "L"))  # store as attribute to prevent GC
    canvas.create_image(0, 0, image=window.image, anchor=tk.NW)

    # draw origin
    origin = camera.project(np.array([[0, 0, 0]]).T).T[0] + 200
    canvas.create_oval(origin[0], origin[1],
                       origin[0] + 8, origin[1] + 8, fill="red")

    print(
        f"Whole render took {1000 * (time.perf_counter() - start):4.0f} ms\n")


def key_press(event):
    global camera

    if event.char == "w":
        camera.pos[2, 0] += -1
    if event.char == "s":
        camera.pos[2, 0] += 1
    if event.char == "a":
        camera.pos[0, 0] += -1
    if event.char == "d":
        camera.pos[0, 0] += 1
    if event.char == "q":
        camera.roll += -0.2
    if event.char == "e":
        camera.roll += 0.2
    if event.char == "r":
        camera.pitch += -0.2
    if event.char == "f":
        camera.pitch += 0.2

    render_PIL(canvas)


def spinning_animation():
    global camera
    camera.yaw += 0.03
    render_PIL(canvas)
    window.after(150, spinning_animation)


window = tk.Tk()
window.title("Projection Matrices")
canvas = tk.Canvas(width=400, height=400, bg="white")
canvas.pack()
render_PIL(canvas)
window.bind("w", key_press)
window.bind("s", key_press)
window.bind("a", key_press)
window.bind("d", key_press)
window.bind("q", key_press)
window.bind("e", key_press)
window.bind("r", key_press)
window.bind("f", key_press)
window.after(150, spinning_animation)
window.mainloop()
