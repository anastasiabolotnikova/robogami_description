import sys
import yaml
import numpy as np
from pathlib import Path

# Packages for visualization
import matplotlib.pyplot as plt
from math import tan, radians, sqrt
from mpl_toolkits.mplot3d import Axes3D

# Packages for generating mesh
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
from stl import mesh

# Booleans for debugging
show_vertices = False

# Vertices visualization function
def plot_vertices_3d(side_vertices):
    x, y, z = side_vertices[:, 0], side_vertices[:, 1], side_vertices[:, 2]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(x, y, z)

    # Add vertex index to each point
    for i in range(len(side_vertices)):
        ax.text(x[i], y[i], z[i], str(i))

    plt.show()

# Mesh STL file generating function
def save_stl(side_vertices, side_faces, filename):
    leg = mesh.Mesh(np.zeros(side_faces.shape[0], dtype=mesh.Mesh.dtype))

    for i, f in enumerate(side_faces):
        for j in range(3):
            leg.vectors[i][j] = side_vertices[f[j], :]

    leg.save(filename)

# Check if a valid configuration file is passed as an argument
if len(sys.argv) != 2 or not Path(sys.argv[1]).is_file() or not sys.argv[1].endswith((".yaml", ".yml")):
    sys.exit(f"Input config file error! Please specify a valid YAML file as an argument. Usage: {sys.argv[0]} <config.yaml>")

# Open config file
config_file = sys.argv[1]
with open(config_file, "r") as f:
    data = yaml.safe_load(f)

# Load variables
vars_to_import = ['hexagon_side', 'hexagon_thickness', 'leg_width', 'leg_length', 'leg_angle', 'leg_thickness']
missing = [v for v in vars_to_import if v not in data]
if missing:
    raise KeyError(f"Missing keys in YAML config file: {missing}")
base_a, base_t, side_a, side_H, side_ang, side_c = (data[v] for v in vars_to_import) 

# Compute auxiliary dimensions
side_h = (side_a / 2.0) * tan(radians(side_ang))
side_b = side_H - side_h
base_r = sqrt(3.0)/2.0 * base_a

# Define 12 3D vertices of a hexagonal base shape
base_vertices = np.array([\
    [base_r, -base_a/2.0, base_t/2.0], # 0
    [base_r, base_a/2.0, base_t/2.0], # 1
    [0, base_a, base_t/2.0], # 2
    [-base_r, base_a/2.0, base_t/2.0], # 3
    [-base_r, -base_a/2.0, base_t/2.0], # 4
    [0, -base_a, base_t/2.0], # 5
    [base_r, -base_a/2.0, -base_t/2.0], # 6
    [base_r, base_a/2.0, -base_t/2.0], # 7
    [0, base_a, -base_t/2.0], # 8
    [-base_r, base_a/2.0, -base_t/2.0], # 9
    [-base_r, -base_a/2.0, -base_t/2.0], # 10
    [0, -base_a, -base_t/2.0], # 11
    ])

# Define 20 triangular faces for the hexagonal base shape using indexes of the vertices
# Note, each triangle is defined using vertice indexes in a specific order to create a valid mesh
base_faces = np.array([[0,1,2], [0,2,3], [0,3,4], [0,4,5], [7,6,11], [7,11,10], [7,10,9], [7,9,8], [0,6,7], [0,7,1], [1,7,2], [7,8,2], [2,8,3], [8,9,3], [10,4,3], [10,3,9], [11,6,0], [11,0,5], [10,11,5], [10,5,4]])

# Define 10 3D vertices of a leg shape
side_vertices = np.array([\
    [0, -side_a/2.0, side_c/2.0], # 0
    [0, side_a/2.0, side_c/2.0], # 1
    [-side_b, side_a/2.0, side_c/2.0], # 2
    [-side_b, -side_a/2.0, side_c/2.0], # 3
    [0, -side_a/2.0, -side_c/2.0], # 4
    [0, side_a/2.0, -side_c/2.0], # 5
    [-side_b, side_a/2.0, -side_c/2.0], # 6
    [-side_b, -side_a/2.0, -side_c/2.0], # 7
    [-side_b-side_h, 0, side_c/2.0], # 8
    [-side_b-side_h, 0, -side_c/2.0]]) # 9

# Visualize the vertices
if show_vertices:
    plot_vertices_3d(base_vertices)
    plot_vertices_3d(side_vertices)

# Define 16 triangular faces for the leg shape mesh using indexes of the vertices
side_faces = np.array([[0,1,3], [1,2,3], [1,5,2], [5,6,2], [5,4,7], [5,7,6], [4,0,7], [3,7,0], [0,5,1], [0,4,5], [2,6,8], [6,9,8], [7,3,8], [7,8,9], [3,2,8], [6,7,9]])

# Generate the base and leg mesh files
save_stl(base_vertices, base_faces, '../meshes/base.stl')
save_stl(side_vertices, side_faces, '../meshes/leg.stl')

# Write the URDF file (define links and joints)
suffix = Path(config_file).stem.replace("robogami_config_", "", 1)
urdf_file = "robogami.urdf" if suffix == "baseline" else f"robogami_{suffix}.urdf"
urdf = open("../urdf/"+urdf_file, "w")

# URDF general info
urdf.write('<?xml version="1.0" ?>\n')
urdf.write('<robot name="Robogami" xmlns:xacro="http://www.ros.org/wiki/xacro">\n')
urdf.write('\t<material name="Gray">\n')
urdf.write('\t\t<color rgba="0.1 0.1 0.1 1.0"/>\n')
urdf.write('\t</material>\n\n')

# Link URDF string generating function
def generate_link_urdf_str(link_name, mesh_name, offset=0):
    return "\n".join([
        '\t<link name="{}">'.format(link_name),
        '\t\t<inertial>',
        '\t\t\t<mass value="0.01" />',
        '\t\t\t<origin rpy="0 0 0" xyz="0 0 0"/>',
        '\t\t\t<inertia ixx="1" ixy="0.0" ixz="0.0" iyy="1" iyz="0.0" izz="1"/>',
        '\t\t</inertial>',## TODO set correct link mass/inertias (for all links) if care about torques
        '\t\t<visual>',
        '\t\t\t<origin rpy="0 0 0" xyz="{} 0 0"/>'.format(str(offset)),
        '\t\t\t<geometry>',
        '\t\t\t\t<mesh filename="package://robogami_description/meshes/{}.stl"/>'.format(mesh_name),
        '\t\t\t</geometry>',
        '\t\t\t<material name="Gray"/>',
        '\t\t</visual>',
        '\t</link>\n\n'
    ])
    
# Joint URDF string generating function
def generate_joint_urdf_str(joint_name, parent_link, child_link, ll, ul, axis, rpy, xyz):
    return "\n".join([
        '\t<joint name="{}" type="revolute">'.format(joint_name),
        '\t\t<parent link="{}"/>'.format(parent_link),
        '\t\t<child link="{}"/>'.format(child_link),
        '\t\t<axis xyz="{}"/>'.format(axis),
        '\t\t<limit effort="1" lower="{}" upper="{}" velocity="1"/>'.format(str(ll), str(ul)), # TODO double check joint limits
        '\t\t<origin rpy="{} {} {}" xyz="{} {} {}"/>'.format(*rpy, *xyz),
        '\t</joint>\n\n'
    ])

# Spherical joint URDF string generating function
def generate_spherical_joint_urdf_str(parent_link, child_link, extra_link1, extra_link2, extra_joint1, extra_joint2, extra_joint3):
    return "\n".join([
        '\t<link name="{}"/>\n'.format(extra_link1), # two virtual links
        '\t<link name="{}"/>\n'.format(extra_link2),
        '\t<joint name="{}" type="revolute">'.format(extra_joint1), # X rotation joint
        '\t\t<parent link="{}"/>'.format(parent_link),
        '\t\t<child link="{}"/>'.format(extra_link1),
        '\t\t<axis xyz="1 0 0"/>',
        '\t\t<limit effort="1" lower="-1.56" upper="1.56" velocity="1"/>',
        '\t\t<origin rpy="0 {} 0" xyz="{} 0 0"/>'.format(radians(180), -side_H),
        '\t</joint>\n',
        '\t<joint name="{}" type="revolute">'.format(extra_joint2), # Y rotation joint
        '\t\t<parent link="{}"/>'.format(extra_link1),
        '\t\t<child link="{}"/>'.format(extra_link2),
        '\t\t<axis xyz="0 1 0"/>',
        '\t\t<limit effort="1" lower="0.1" upper="3.13" velocity="1"/>',
        '\t\t<origin rpy="0 0 0" xyz="0 0 0"/>',
        '\t</joint>\n',
        '\t<joint name="{}" type="revolute">'.format(extra_joint3), # Z ?? rotation joint
        '\t\t<parent link="{}"/>'.format(extra_link2),
        '\t\t<child link="{}"/>'.format(child_link),
        '\t\t<axis xyz="1 0 0"/>',
        '\t\t<limit effort="1" lower="-1.56" upper="1.56" velocity="1"/>',
        '\t\t<origin rpy="0 0 0" xyz="0 0 0"/>',
        '\t</joint>\n\n'
    ])

# Base hexagon link
urdf.write(generate_link_urdf_str("base", "base"))

# Legs lower links
urdf.write(generate_link_urdf_str("leg1", "leg"))
urdf.write(generate_link_urdf_str("leg2", "leg"))
urdf.write(generate_link_urdf_str("leg3", "leg"))

# Joints connecting leg lower links to the hexagon base link (actuated DoFs)
urdf.write(generate_joint_urdf_str("l1", "base", "leg1", 0, 1.4, "0 1 0", [0, 0, 0], [-base_r, 0, 0]))
urdf.write(generate_joint_urdf_str("l2", "base", "leg2", 0, 1.4, "0 1 0", [0, 0, radians(120.0)], [base_a/4.0*sqrt(3), -3.0*base_a/4.0, 0]))
urdf.write(generate_joint_urdf_str("l3", "base", "leg3", 0, 1.4, "0 1 0", [0, 0, radians(-120.0)], [base_a/4.0*sqrt(3), 3.0*base_a/4.0, 0]))

# Legs upper links
urdf.write(generate_link_urdf_str("leg1top", "leg", side_H))
urdf.write(generate_link_urdf_str("leg2top", "leg", side_H))
urdf.write(generate_link_urdf_str("leg3top", "leg", side_H))

# Spherical joints connecting leg lower and upper links (unactuated DoFs)
urdf.write(generate_spherical_joint_urdf_str("leg1", "leg1top", "l1RotX", "l1RotY", "x_l1rotx", "x_l1roty", "x_l1top"))
urdf.write(generate_spherical_joint_urdf_str("leg2", "leg2top", "l2RotX", "l2RotY", "x_l2rotx", "x_l2roty", "x_l2top"))
urdf.write(generate_spherical_joint_urdf_str("leg3", "leg3top", "l3RotX", "l3RotY", "x_l3rotx", "x_l3roty", "x_l3top"))

# Top hexagon link
urdf.write(generate_link_urdf_str("top", "base", -base_r))

# Leg1 joint to top hexagon link
urdf.write(generate_joint_urdf_str("l1topBase", "leg1top", "top", 0, 1.56, "0 1 0", [0, radians(180.0), 0], [side_H, 0, 0]))

# Done!
urdf.write('\n</robot>')
urdf.close()