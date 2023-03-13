import xml.etree.ElementTree as ET
from xml.dom import minidom
import numpy as np
from pathlib import Path

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
    
def add_wheels(robot, wheels):
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
            raise Exception(f'RW{i + 1} is too close to scope')
    
    # Check if wheels are too close to each other
    for i, w_i in enumerate(wheels):
        for j, w_j in enumerate(wheels):
            if j <= i:
                continue
            if np.linalg.norm(w_i['xyz'] - w_j['xyz']) <= min_dist_rw_to_rw:
                raise Exception(f'RW{i + 1} is too close to RW{j + 1}')
    
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