import numpy as np
from PIL import Image
import yaml

from pyglet.gl import *
import pyglet
from pyglet import font, graphics, window

import argparse

class Camera:
    """ A simple 2D camera that contains the speed and offset."""

    def __init__(self, window: pyglet.window.Window, scroll_speed=1, min_zoom=1, max_zoom=4):
        assert min_zoom <= max_zoom, "Minimum zoom must not be greater than maximum zoom"
        self._window = window
        self.scroll_speed = scroll_speed
        self.max_zoom = max_zoom
        self.min_zoom = min_zoom
        self.offset_x = 0
        self.offset_y = 0
        self._zoom = max(min(1, self.max_zoom), self.min_zoom)

    @property
    def zoom(self):
        return self._zoom

    @zoom.setter
    def zoom(self, value):
        """ Here we set zoom, clamp value to minimum of min_zoom and max of max_zoom."""
        self._zoom = max(min(value, self.max_zoom), self.min_zoom)

    @property
    def position(self):
        """Query the current offset."""
        return self.offset_x, self.offset_y

    @position.setter
    def position(self, value):
        """Set the scroll offset directly."""
        self.offset_x, self.offset_y = value

    def move(self, axis_x, axis_y):
        """ Move axis direction with scroll_speed.
            Example: Move left -> move(-1, 0)
         """
        self.offset_x += self.scroll_speed * axis_x
        self.offset_y += self.scroll_speed * axis_y

    def begin(self):
        # Set the current camera offset so you can draw your scene.

        # Translate using the offset.
        view_matrix = self._window.view.translate(-self.offset_x * self._zoom, -self.offset_y * self._zoom, 0)
        # Scale by zoom level.
        view_matrix = view_matrix.scale(self._zoom, self._zoom, 1)

        self._window.view = view_matrix

    def end(self):
        # Since this is a matrix, you will need to reverse the translate after rendering otherwise
        # it will multiply the current offset every draw update pushing it further and further away.

        # Reverse scale, since that was the last transform.
        view_matrix = self._window.view.scale(1 / self._zoom, 1 / self._zoom, 1)
        # Reverse translate.
        view_matrix = view_matrix.translate(self.offset_x * self._zoom, self.offset_y * self._zoom, 0)

        self._window.view = view_matrix

    def __enter__(self):
        self.begin()

    def __exit__(self, exception_type, exception_value, traceback):
        self.end()


class CenteredCamera(Camera):
    """A simple 2D camera class. 0, 0 will be the center of the screen, as opposed to the bottom left."""

    def begin(self):
        x = -self._window.width // 2 / self._zoom + self.offset_x
        y = -self._window.height // 2 / self._zoom + self.offset_y

        view_matrix = self._window.view.translate(-x * self._zoom, -y * self._zoom, 0)
        view_matrix = view_matrix.scale(self._zoom, self._zoom, 1)
        self._window.view = view_matrix

    def end(self):
        x = -self._window.width // 2 / self._zoom + self.offset_x
        y = -self._window.height // 2 / self._zoom + self.offset_y

        view_matrix = self._window.view.scale(1 / self._zoom, 1 / self._zoom, 1)
        view_matrix = view_matrix.translate(x * self._zoom, y * self._zoom, 0)
        self._window.view = view_matrix



parser = argparse.ArgumentParser()
parser.add_argument('--map_path', type=str, required=True, help='Path to the map without extensions')
parser.add_argument('--map_ext', type=str, required=True, help='Extension of the map image file')
args = parser.parse_args()

# load map yaml
with open(args.map_path + '.yaml', 'r') as yaml_stream:
    try:
        map_metada = yaml.safe_load(yaml_stream)
        map_resolution = map_metada['resolution']
        origin = map_metada['origin']
        origin_x = origin[0]
        origin_y = origin[1]
    except yaml.YAMLError as ex:
        print(ex)

# load map image
map_img = np.array(Image.open(args.map_path + args.map_ext).transpose(Image.FLIP_TOP_BOTTOM)).astype(np.float64)
map_height = map_img.shape[0]
map_width = map_img.shape[1]

# convert map pixels to coordinates
range_x = np.arange(map_width)
range_y = np.arange(map_height)
map_x, map_y = np.meshgrid(range_x, range_y)
map_x = (map_x * map_resolution + origin_x).flatten()
map_y = (map_y * map_resolution + origin_y).flatten()
map_z = np.zeros(map_y.shape)
map_coords = np.vstack((map_x, map_y, map_z))

# mask and only leave the obstacle points
map_mask = map_img == 0.0
map_mask_flat = map_mask.flatten()
map_points = map_coords[:, map_mask_flat].T

# prep opengl
try:
    # Try and create a window with multisampling (antialiasing)
    config = Config(sample_buffers=1, samples=4,
                    depth_size=16, double_buffer=True, )
    window = window.Window(resizable=True, config=config)
except window.NoSuchConfigException:
    # Fall back to no multisampling for old hardware
    window = window.Window(resizable=True)

glClearColor(18/255, 4/255, 88/255, 1.)
glEnable(GL_DEPTH_TEST)
glTranslatef(25, -5, -60)

cam = Camera(window)

@window.event
def on_resize(width, height):
    # Override the default on_resize handler to create a 3D projection
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60., width / float(height), .1, 1000.)
    glMatrixMode(GL_MODELVIEW)
    return pyglet.event.EVENT_HANDLED

batch = graphics.Batch()

points = []
for i in range(map_points.shape[0]):
    particle = batch.add(1, GL_POINTS, None, ('v3f/stream', [map_points[i, 0], map_points[i, 1], map_points[i, 2]]))
    points.append(particle)

def loop(dt):
    print(pyglet.clock.get_fps())
    pass

@window.event
def on_draw():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glColor3f(254/255, 117/255, 254/255)
    cam.begin()
    batch.draw()
    cam.end()

pyglet.clock.schedule(loop)
pyglet.app.run()