#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Vec2D(object):
  def __init__(self, x=1.0, y=0.0):
    self.x = x
    self.y = y
    self.angle = 0.0
    self.length = 0.0
    self.update()

  def setVector(self, alpha=0.0, length=0.0):
    self.x = np.cos(alpha)
    self.y = np.sin(alpha)
    self.normalize()
    self.x *= length
    self.y *= length
    self.update()

  def normalize(self):
    self.update()
    self.x = self.x / self.length
    self.y = self.y / self.length
    self.update()

  def update(self):
    self.length = np.sqrt((self.x * self.x) + (self.y * self.y))
    self.angle = np.arctan2(self.y, self.x)

  def print(self, prefix=""):
    print(prefix + " x: %.2f y: %.2f angle: %.2f length: %.2f" % (self.x, self.y, ((self.angle/np.pi)*180.0), self.length))

  def plot(self, vec_orig=None, color='b'):
    if vec_orig is None:
      plt.quiver([0.0], [0.0], [self.x], [self.y], angles='xy', scale_units='xy', scale=1, color=color, width=0.006)
    else:
      plt.quiver([vec_orig.x], [vec_orig.y], [self.x], [self.y], angles='xy', scale_units='xy', scale=1, color=color, width=0.1)

  def __add__(self, other):
    return Vec2D(self.x + other.x, self.y + other.y)

def deg_to_rad(deg):
  """
  Convert degree to rad 

  Parameters:
  deg: Angle in degree

  Return:
  Angle in rad
  """
  return (deg / 180.0) * np.pi

def calcSVPMVecLengths(angle_deg, V_DC=1.0, V_Mot=0.5):

  # Calculate angle and limit it
  angle_rad = deg_to_rad(angle_deg % 60.0)

  # Length V depends on the modulation index
  l_v = V_Mot / V_DC # This index shall be smaller than 0.83

  # Calculate the lengths of the vectors
  length_a = (2.0 / np.sqrt(3.0)) * l_v * np.sin((np.pi/3.0) - angle_rad)
  length_b = (2.0 / np.sqrt(3.0)) * l_v * np.sin(angle_rad)

  if length_a > 1.0: length_a = 1.0
  elif length_a < 0.0: length_a = 0.0

  if length_b > 1.0: length_b = 1.0
  elif length_b < 0.0: length_b = 0.0

  return length_a, length_b


# Create plots
fig, ax = plt.subplots(1)

# Create circle 
circle1 = plt.Circle( ( 0, 0 ), 1, edgecolor="black", fill=0 )
ax.add_patch( circle1 )

# Create sector lines
for angle_deg in range(0, 360, 60):
  angle_rad = (angle_deg / 180.0) * np.pi;
  ax.plot([0.0, np.cos(angle_rad)], [0.0, np.sin(angle_rad)], color="black", linestyle="dashed")

# Create vectors
vec_a = Vec2D()
vec_a.setVector(deg_to_rad(0.0), 1.0)

vec_b = Vec2D()
vec_b.setVector(deg_to_rad(60.0), 1.0)

vec_z = Vec2D()
vec_z.setVector(deg_to_rad(0.0), 0.5)

plt_vec_a = ax.quiver([0.0], [0.0], [vec_a.x], [vec_a.y], angles='xy', scale_units='xy', scale=1, color='r', width=0.01)
plt_vec_b = ax.quiver([0.0], [0.0], [vec_b.x], [vec_b.y], angles='xy', scale_units='xy', scale=1, color='g', width=0.01)
plt_vec_z = ax.quiver([0.0], [0.0], [vec_z.x], [vec_z.y], angles='xy', scale_units='xy', scale=1, color='b', width=0.01)

# Create point list
x_points = [0]
y_points = [0]
plt_points, = ax.plot(x_points, y_points, color='b')

def anim_run(i):
  global x_points, y_points

  angle_deg = (0.4 * float(i)) % 360.0;

  length_a, length_b = calcSVPMVecLengths(angle_deg, 1.0, 1.0)

  if angle_deg >= 0.0 and angle_deg < 60.0:
    vec_a.setVector(deg_to_rad(0.0), length_a)
    vec_b.setVector(deg_to_rad(60.0), length_b)
    vec_c = vec_a + vec_b
  elif angle_deg >= 60.0 and angle_deg < 120.0:
    vec_a.setVector(deg_to_rad(60.0), length_a)
    vec_b.setVector(deg_to_rad(120.0), length_b)
    vec_c = vec_a + vec_b
  elif angle_deg >= 120.0 and angle_deg < 180.0:
    vec_a.setVector(deg_to_rad(120.0), length_a)
    vec_b.setVector(deg_to_rad(180.0), length_b)
    vec_c = vec_a + vec_b
  elif angle_deg >= 180.0 and angle_deg < 240.0:
    vec_a.setVector(deg_to_rad(180.0), length_a)
    vec_b.setVector(deg_to_rad(240.0), length_b)
    vec_c = vec_a + vec_b
  elif angle_deg >= 240.0 and angle_deg < 300.0:
    vec_a.setVector(deg_to_rad(240.0), length_a)
    vec_b.setVector(deg_to_rad(300.0), length_b)
    vec_c = vec_a + vec_b
  elif angle_deg >= 300.0 and angle_deg < 360.0:
    vec_a.setVector(deg_to_rad(300.0), length_a)
    vec_b.setVector(deg_to_rad(0.0), length_b)
    vec_c = vec_a + vec_b

  plt_vec_a.set_UVC(vec_a.x, vec_a.y)
  plt_vec_b.set_UVC(vec_b.x, vec_b.y)
  plt_vec_z.set_UVC(vec_c.x, vec_c.y)

  x_points.append(vec_c.x)
  y_points.append(vec_c.y)
  plt_points.set_xdata(x_points)
  plt_points.set_ydata(y_points)

  return plt_vec_a, plt_vec_b, plt_vec_z, plt_points

fig = plt.gcf()

ani = animation.FuncAnimation(
    fig, anim_run, interval=10, blit=True, save_count=50)

ax.set_aspect('equal')
plt.xlim(-1.2, 1.2)
plt.ylim(-1.2, 1.2)
plt.grid(True)
plt.show()