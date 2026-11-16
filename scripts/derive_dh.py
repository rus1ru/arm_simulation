#!/usr/bin/python3
"""Derive standard DH from actual URDF axis lines; verify against original FK."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from gp7_kinematics import Kinematics
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SIM_URDF = ROOT / 'urdf/robo2_sim_geometry.urdf'


def transform(r=None, p=None):
    t = np.eye(4)
    if r is not None:
        t[:3,:3] = r
    if p is not None:
        t[:3,3] = p
    return t


def derive():
    k = Kinematics(SIM_URDF)
    sim = ET.parse(SIM_URDF).getroot()
    tool_joint = sim.find("joint[@name='tool0_fixed']")
    tool_origin = tool_joint.find('origin')
    tcp = transform(
        Rotation.from_euler('xyz', np.fromstring(tool_origin.get('rpy', '0 0 0'), sep=' ')).as_matrix(),
        np.fromstring(tool_origin.get('xyz', '0 0 0'), sep=' '))
    t = transform(p=[0, 0, .002])
    lines = []
    for origin, axis in zip(k.origins, k.axes):
        t = t @ origin
        lines.append((t[:3,3].copy(), t[:3,:3] @ axis))
    p, z = lines[0]
    x = np.array([1., 0, 0])
    x -= z * np.dot(z, x)
    x /= np.linalg.norm(x)
    frames = [transform(np.column_stack((x, np.cross(z,x), z)), p)]
    for (p,z), (pn,zn) in zip(lines, lines[1:]):
        # Closest points p+s*z and pn+t*zn on the two axis lines.
        # All consecutive exported axes here are nonparallel.
        cross = np.cross(z, zn)
        if np.linalg.norm(cross) < 1e-10:
            raise ValueError('Parallel-axis frame construction not implemented')
        s, u = np.linalg.lstsq(np.column_stack((z, -zn)), pn-p, rcond=None)[0]
        x = cross / np.linalg.norm(cross)
        on_next = pn + u*zn
        frames.append(transform(np.column_stack((x, np.cross(zn,x), zn)), on_next))
    frames.append(frames[-1].copy())
    rows = []
    for before, after in zip(frames, frames[1:]):
        delta = np.linalg.inv(before) @ after
        theta = np.arctan2(delta[1,0], delta[0,0])
        alpha = np.arctan2(delta[2,1], delta[2,2])
        d = delta[2,3]
        a = np.dot(delta[:3,3], delta[:3,0])
        rows.append([float(theta), float(d), float(a), float(alpha)])
    tool = np.linalg.inv(frames[-1]) @ k.fk(np.zeros(6)) @ np.linalg.inv(tcp)
    return k, frames[0], rows, tool, tcp


def dh_fk(q, base, rows, tool, tcp):
    t = base.copy()
    for angle, (offset,d,a,alpha) in zip(q,rows):
        t = t @ transform(Rotation.from_rotvec([0,0,angle+offset]).as_matrix())
        t = t @ transform(p=[0,0,d]) @ transform(p=[a,0,0])
        t = t @ transform(Rotation.from_rotvec([alpha,0,0]).as_matrix())
    return t @ tool @ tcp


def verify(k, base, rows, tool, tcp, samples=10000):
    rng = np.random.default_rng(20261115)
    position_error = rotation_error = 0.
    for q in np.vstack((np.zeros(6), rng.uniform(k.lower,k.upper,(samples,6)))):
        actual = k.fk(q)
        derived = dh_fk(q,base,rows,tool,tcp)
        position_error = max(position_error, np.linalg.norm(actual[:3,3]-derived[:3,3]))
        rotation_error = max(rotation_error, Rotation.from_matrix(actual[:3,:3] @ derived[:3,:3].T).magnitude())
    assert position_error < 1e-6 and rotation_error < 1e-6
    return position_error, rotation_error

if __name__ == '__main__':
    k, base, rows, tool, tcp = derive()
    pos, rot = verify(k,base,rows,tool,tcp)
    print(json.dumps(dict(convention='Rz(q+offset) Tz(d) Tx(a) Rx(alpha)',
                         columns=['theta_offset_rad','d_m','a_m','alpha_rad'],
                         rows=rows, world_to_DH0=base.tolist(), DH6_to_Link6=tool.tolist(), Link6_to_current_TCP=tcp.tolist(),
                         tested_configurations=10001, max_position_error_m=pos,
                         max_rotation_error_rad=rot), indent=2))
