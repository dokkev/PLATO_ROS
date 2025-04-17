import os
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def parse_urdf(urdf_file):
  """Parses a URDF file (xacro format) and extracts robot link positions.

  Args:
    urdf_file: Path to the URDF file.

  Returns:
    A list of dictionaries containing link position information.
  """
  tree = ET.parse(urdf_file)
  root = tree.getroot()

  links = []
  for link_element in root.iter('link'):
    link_name = link_element.get('name')
    origin = link_element.find('visual/origin')
    if origin is not None:
      x = float(origin.get('xyz').split()[0])
      y = float(origin.get('xyz').split()[1])
      z = float(origin.get('xyz').split()[2])
      links.append({
          'name': link_name,
          'position': np.array([x, y, z])
      })
  return links

def visualize_workspace(links):
  """Visualizes the robot links as bars in 3D space.

  Args:
    links: A list of dictionaries containing link position information.
  """
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')

  for i in range(len(links) - 1):
    start_pos = links[i]['position']
    end_pos = links[i + 1]['position']
    ax.plot([start_pos[0], end_pos[0]],
              [start_pos[1], end_pos[1]],
              [start_pos[2], end_pos[2]],
              color='blue')

  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  plt.show()

if __name__ == "__main__":
  urdf_file = 'plato_hand.urdf.xacro'  # Replace with your URDF file path
  links = parse_urdf(urdf_file)
  visualize_workspace(links)