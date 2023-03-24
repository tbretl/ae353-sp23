import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
from pathlib import Path
import meshcat

# Minimum distance between the center of wheel pairs
min_dist_rw_to_rw = np.sqrt(0.5**2 + 0.1**2) + np.sqrt(0.5**2 + 0.1**2) + 0.001

# Minimum distance between center of each wheel and center of scope
min_dist_rw_to_scope = np.sqrt(0.5**2 + 0.1**2) + np.sqrt(0.8**2 + 0.1**2) + 0.001

# Distance between center of spacecraft and center of each wheel
r = 2.2

def add_material(robot, name, rgba):
    mat = ET.SubElement(robot, 'material', attrib={'name': name})
    col = ET.SubElement(mat, 'color', attrib={'rgba': ' '.join(str(v) for v in rgba)})

def add_link(robot, name, stl, scale, mass, inertia, material, concave=True):
    link = ET.SubElement(robot, 'link', attrib={'name': name, 'concave': f'{"yes" if concave else "no"}'})
    vis = ET.SubElement(link, 'visual')
    geo = ET.SubElement(vis, 'geometry')
    ET.SubElement(
        geo,
        'mesh',
        attrib={
            'filename': stl,
            'scale': ' '.join(str(v) for v in scale),
        },
    )
    ET.SubElement(vis, 'material', attrib={'name': material})
    col = ET.SubElement(link, 'collision', attrib={'concave': f'{"yes" if concave else "no"}'})
    geo = ET.SubElement(col, 'geometry')
    ET.SubElement(
        geo,
        'mesh',
        attrib={
            'filename': stl,
            'scale': ' '.join(str(v) for v in scale),
        },
    )
    inertial = ET.SubElement(link, 'inertial')
    ET.SubElement(inertial, 'origin', attrib={
        'rpy': ' '.join(str(v) for v in [0., 0., 0.]),
        'xyz': ' '.join(str(v) for v in [0., 0., 0.]),
    })
    ET.SubElement(inertial, 'mass', attrib={'value': f'{mass}'})
    ET.SubElement(inertial, 'inertia', attrib={
        'ixx': f'{inertia[0]}',
        'ixy': f'{inertia[1]}',
        'ixz': f'{inertia[2]}',
        'iyy': f'{inertia[3]}',
        'iyz': f'{inertia[4]}',
        'izz': f'{inertia[5]}',
    })

def add_joint(robot, props):
    joint = ET.SubElement(robot, 'joint', attrib={
        'name': f'{props["parent"]}_to_{props["child"]}',
        'type': props['type'],
    })
    ET.SubElement(joint, 'parent', attrib={'link': props['parent']})
    ET.SubElement(joint, 'child', attrib={'link': props['child']})
    ET.SubElement(joint, 'origin', attrib={
        'xyz': ' '.join(str(v) for v in props['origin']['xyz']),
        'rpy': ' '.join(str(v) for v in props['origin']['rpy']),
    })
    ET.SubElement(joint, 'axis', attrib={'xyz': ' '.join(str(v) for v in props['axis']['xyz'])})
    ET.SubElement(joint, 'limit', attrib={
        'effort': f'{props["limit"]["effort"]}',
        'velocity': f'{props["limit"]["velocity"]}',
    })

def wheels_are_valid(wheels):
    valid = True

    # Find xyz and rpy of each wheel
    for w in wheels:
        alpha = w['alpha']
        delta = w['delta']
        w['xyz'] = r * np.array([np.cos(alpha) * np.cos(delta), np.sin(alpha) * np.cos(delta), np.sin(delta)])
        w['rpy'] = [(np.pi / 2) - delta, 0., (np.pi / 2) + alpha]
    
    # Find xyz of scope
    scope_xyz = r * np.array([1., 0., 0.])
    
    # Check if wheels are too close to scope
    for i, w in enumerate(wheels):
        if np.linalg.norm(w['xyz'] - scope_xyz) <= min_dist_rw_to_scope:
            print(f'WARNING: RW{i + 1} is too close to scope')
            valid = False
    
    # Check if wheels are too close to each other
    for i, w_i in enumerate(wheels):
        for j, w_j in enumerate(wheels):
            if j <= i:
                continue
            if np.linalg.norm(w_i['xyz'] - w_j['xyz']) <= min_dist_rw_to_rw:
                print(f'WARNING: RW{i + 1} is too close to RW{j + 1}')
                valid = False
    
    return valid

def add_wheels(robot, wheels):
    if not wheels_are_valid(wheels):
        raise Exception('Invalid placement of reaction wheels')

    # Add wheels to URDF
    for i, w in enumerate(wheels):
        add_joint(robot, {
            'type': 'continuous',
            'parent': 'bus',
            'child': f'wheel_{i + 1}',
            'origin': {'xyz': w['xyz'], 'rpy': w['rpy']},
            'axis': {'xyz': [0., 0., 1.]},
            'limit': {'effort': 1000., 'velocity': 1000.},
        })

def create_spacecraft(wheels, urdf='spacecraft.urdf'):
    robot = ET.Element('robot', attrib={'name': 'spacecraft'})
    add_material(robot, 'industrial-blue', [0.11372549019607843, 0.34509803921568627, 0.6549019607843137, 1.])
    add_material(robot, 'arches-blue', [0., 0.6235294117647059, 0.8313725490196079, 1.])
    add_material(robot, 'heritage-orange', [0.96078431, 0.50980392, 0.11764706, 1.])
    add_link(robot, 'bus', 'spacecraft.stl', [1., 1., 1.], 6., [10., 0., 0., 10., 0., 16.], 'industrial-blue')
    add_link(robot, 'wheel_1', 'rw1.stl', [0.5, 0.5, 0.2], 1., [0.075, 0., 0., 0.075, 0., 0.125], 'heritage-orange')
    add_link(robot, 'wheel_2', 'rw2.stl', [0.5, 0.5, 0.2], 1., [0.075, 0., 0., 0.075, 0., 0.125], 'heritage-orange')
    add_link(robot, 'wheel_3', 'rw3.stl', [0.5, 0.5, 0.2], 1., [0.075, 0., 0., 0.075, 0., 0.125], 'heritage-orange')
    add_link(robot, 'wheel_4', 'rw4.stl', [0.5, 0.5, 0.2], 1., [0.075, 0., 0., 0.075, 0., 0.125], 'heritage-orange')
    add_wheels(robot, wheels)
    xmlstr = minidom.parseString(ET.tostring(robot)).toprettyxml(indent="  ")
    with open(Path(f'./urdf/{urdf}'), 'w') as f:
        f.write(xmlstr)

def convert_color(rgba):
    color = int(rgba[0] * 255) * 256**2 + int(rgba[1] * 255) * 256 + int(rgba[2] * 255)
    opacity = rgba[3]
    transparent = opacity != 1.0
    return {
        'color': color,
        'opacity': opacity,
        'transparent': transparent,
    }

def create_visualizer():
    # Create visualizer
    vis = meshcat.Visualizer()

    # Create spacecraft
    color = convert_color([0.11372549019607843, 0.34509803921568627, 0.6549019607843137, 1.])
    vis['spacecraft'].set_object(
        meshcat.geometry.StlMeshGeometry.from_file(Path('./urdf/spacecraft.stl')),
        meshcat.geometry.MeshPhongMaterial(
            color=color['color'],
            transparent=color['transparent'],
            opacity=color['opacity'],
            reflectivity=0.8,
        )
    )

    # Create wheels
    color = convert_color([0.96078431, 0.50980392, 0.11764706, 1.])
    for i in range(4):
        vis[f'rw{i + 1}'].set_object(
            meshcat.geometry.StlMeshGeometry.from_file(Path(f'./urdf/rw{i + 1}.stl')),
            meshcat.geometry.MeshPhongMaterial(
                color=color['color'],
                transparent=color['transparent'],
                opacity=color['opacity'],
                reflectivity=0.8,
            )
        )
    
    # Set camera view
    vis['/Cameras/default'].set_transform(
        meshcat.transformations.compose_matrix(
            angles=[
                0.,
                np.deg2rad(-30.),
                np.deg2rad(60. - 180.),
            ],
            translate=[0., 0., 0.],
        )
    )
    vis['/Cameras/default/rotated/<object>'].set_property(
        'position', [5., 0., 0.],
    )
    vis['/Cameras/default/rotated/<object>'].set_property(
        'fov', 90,
    )
    
    # Return visualizer
    return vis

def show_wheels(vis, wheels):
    if not wheels_are_valid(wheels):
        print('WARNING: Invalid placement of reaction wheels')
    
    for i, w in enumerate(wheels):
        S = np.diag(np.concatenate(([0.5, 0.5, 0.2], [1.0])))
        T = meshcat.transformations.euler_matrix(*w['rpy'])
        T[:3, 3] = np.array(w['xyz'])[:3]
        vis[f'rw{i + 1}'].set_transform(T @ S)
