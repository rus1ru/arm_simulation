#!/usr/bin/python3
"""URDF-based numerical kinematics and reproducible pick/place trajectory.

Assumes simulator base origin z=0.002 and a geometry-derived Link_6-to-TCP offset.
Original ungeometry-derived URDF limits are retained; this is not hardware motion code.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]

class Kinematics:
    def __init__(self, urdf=ROOT / 'urdf/robo2.urdf'):
        joints = ET.parse(urdf).getroot().findall('joint')[:6]
        self.names = [j.attrib['name'] for j in joints]
        self.origins = []
        self.axes = []
        self.lower = np.array([float(j.find('limit').attrib['lower']) for j in joints])
        self.upper = np.array([float(j.find('limit').attrib['upper']) for j in joints])
        self.velocity_limits = np.array([float(j.find('limit').attrib['velocity']) for j in joints])
        for j in joints:
            o = j.find('origin')
            t = np.eye(4)
            t[:3, 3] = np.fromstring(o.attrib['xyz'], sep=' ')
            t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(o.attrib['rpy'], sep=' ')).as_matrix()
            self.origins.append(t)
            axis = np.fromstring(j.find('axis').attrib['xyz'], sep=' ')
            self.axes.append(axis / np.linalg.norm(axis))

    def fk(self, q):
        t = np.eye(4)
        t[2, 3] = .002
        for origin, axis, angle in zip(self.origins, self.axes, q):
            r = np.eye(4)
            r[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            t = t @ origin @ r
        t[:3, 3] += t[:3, :3] @ np.array([0.005160158262354511, 0.001412173059851697, 0.10536599580637081])
        return t

    def ik(self, position, orientation, seed):
        def error(q):
            t = self.fk(q)
            return np.r_[t[:3, 3] - position,
                         Rotation.from_matrix(orientation @ t[:3, :3].T).as_rotvec() * .3]
        result = least_squares(error, seed, bounds=(self.lower, self.upper),
                               xtol=1e-11, ftol=1e-11, gtol=1e-11, max_nfev=300)
        if np.linalg.norm(error(result.x)) > 1e-5:
            raise ValueError('IK failed: ' + str(result.fun))
        return result.x

class ArmCollisionChecker:
    """Conservative mesh-local AABBs vs fixed box_a and ground (not mesh/self collision)."""
    def __init__(self, k):
        self.k = k
        self.bounds = []
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        for i in range(1, 7):
            data = (ROOT / f'meshes/Link_{i}.STL').read_bytes()
            count = int.from_bytes(data[80:84], 'little')
            vertices = np.frombuffer(data, dtype=dtype, offset=84, count=count)['vertices'].reshape(-1, 3)
            self.bounds.append((vertices.min(0), vertices.max(0)))

        # Use simulator collision boxes, and transform CAD finger vertices through
        # their collision-origin and prismatic-joint frames (not raw STL bounds).
        model = ET.parse(ROOT / 'urdf/robo2_sim_geometry.urdf').getroot()
        self.finger_bounds = []
        for name in ['Link_7', 'Link_7.2']:
            joint = next(
                j for j in model.findall('joint') if j.find('child').attrib['link'] == name)
            collision = model.find(f"link[@name='{name}']/collision")
            def transform(origin):
                t = np.eye(4)
                t[:3, 3] = np.fromstring(origin.attrib['xyz'], sep=' ')
                t[:3, :3] = Rotation.from_euler('xyz', np.fromstring(origin.attrib['rpy'], sep=' ')).as_matrix()
                return t
            data = (ROOT / f'meshes/{name}.STL').read_bytes()
            vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3)
            vertices = (transform(collision.find('origin')) @ np.c_[vertices, np.ones(len(vertices))].T).T[:, :3]
            self.finger_bounds.append((transform(joint.find('origin')), vertices.min(0), vertices.max(0)))
        palm = model.find("link[@name='Link_6']/collision")
        center = np.fromstring(palm.find('origin').attrib['xyz'], sep=' ')
        half = np.fromstring(palm.find('geometry/box').attrib['size'], sep=' ') / 2
        self.bounds[-1] = (center - half, center + half)

    def clear(self, q, cube=None, finger_position=0.):
        t = np.eye(4)
        t[2, 3] = .002
        blo, bhi = np.array([.35, .35, 0]), np.array([.65, .65, .2])
        bodies = []
        for origin, axis, angle, (lo, hi) in zip(self.k.origins, self.k.axes, q, self.bounds):
            r = np.eye(4)
            r[:3, :3] = Rotation.from_rotvec(axis * angle).as_matrix()
            t = t @ origin @ r
            bodies.append((t.copy(), lo, hi))
        for index, (origin, lo, hi) in enumerate(self.finger_bounds):
            moved = origin.copy()
            moved[:3, 3] += moved[:3, :3] @ np.array([(-1 if index == 0 else 1) * finger_position, 0., 0.])
            bodies.append((t @ moved, lo, hi))
        for t, lo, hi in bodies:
            rotation = t[:3, :3]
            center = rotation @ ((lo + hi) / 2) + t[:3, 3]
            extent = (hi - lo) / 2
            if center[2] - np.sum(extent * np.abs(rotation[2])) < 0:
                return False
            delta = center - (blo + bhi) / 2
            axes = list(np.eye(3)) + list(rotation.T)
            axes += [np.cross(a, b) for a in np.eye(3) for b in rotation.T]
            obstacles = [(blo, bhi)]
            if cube is not None:
                obstacles.append((np.asarray(cube) - .025, np.asarray(cube) + .025))
            for lower, upper in obstacles:
                delta = center - (lower + upper) / 2
                if all(abs(delta @ a) <= np.sum(extent * np.abs(rotation.T @ a))
                       + np.sum((upper - lower) / 2 * np.abs(a)) + 1e-10
                       for a in axes if np.linalg.norm(a) > 1e-9):
                    return False
        return True

def generate():
    k = Kinematics()
    orientation = Rotation.from_euler('xyz', [np.pi, 0, 0]).as_matrix() @ np.array([[-0.00840349577117988, 0.9987682737781275, 0.04890108948769324], [-0.9999553459584186, -0.008604771696726632, 0.00390691607622088], [0.004322886536005939, -0.04886607410368501, 0.9987959849006653]]).T
    grasp_center_offset = np.array([0., 0., .010])
    source = np.array([.4, -.25, .025]) + grasp_center_offset
    object_target = np.array([.5, .5, .225])
    target = object_target + grasp_center_offset
    pre = source + [0, 0, .20]
    endpoints = [('grasp', source), ('lift', source + [0, 0, .30]),
                 ('transfer', target + [0, 0, .10]), ('place', target),
                 ('retreat', target + [0, 0, .10])]
    rng = np.random.default_rng(7)
    checker = ArmCollisionChecker(k)
    base_orientation = orientation.copy()
    solution = None
    # Quarter turns preserve a square box grasp; never change fixed box geometry.
    for yaw in [0., np.pi / 2, -np.pi / 2, np.pi]:
        orientation = Rotation.from_euler('z', yaw).as_matrix() @ base_orientation
        seeds = [np.zeros(6)] + list(rng.uniform(k.lower, k.upper, (120, 6)))
        for seed in seeds:
            try:
                candidate = k.ik(pre, orientation, seed)
                if not all(checker.clear(v * candidate, source - grasp_center_offset) for v in np.linspace(0, 1, 81)):
                    continue
                probe = candidate.copy()
                previous = pre
                for _, end in endpoints:
                    for fraction in np.linspace(0, 1, 41)[1:]:
                        next_probe = k.ik(previous + fraction * (end - previous), orientation, probe)
                        if np.max(np.abs(next_probe - probe)) > .3 or not checker.clear(next_probe, source - grasp_center_offset if _ == 'grasp' else None):
                            raise ValueError('Discontinuous or colliding IK branch')
                        probe = next_probe
                    previous = end
                solution = candidate
                break
            except ValueError:
                pass
        if solution is not None:
            break
    if solution is None:
        raise ValueError('No collision-free continuous full-path IK solution')
    q = solution
    phases = []
    # Quintic timing yields zero endpoint velocity. 0.05s samples, conservative speed.
    def phase(name, qs, duration):
        times = np.linspace(0, duration, len(qs))
        velocities = np.gradient(np.array(qs), times, axis=0)
        velocities[0] = velocities[-1] = 0
        phases.append(dict(name=name, points=[dict(positions=list(a), velocities=list(v),
                          time_from_start=float(t)) for a, v, t in zip(qs, velocities, times)]))
    steps = 161
    u = np.linspace(0, 1, steps)
    s = 10*u**3 - 15*u**4 + 6*u**5
    phase('pregrasp', [v*q for v in s], 8.)
    previous = pre
    for name, end in endpoints:
        qs = [q.copy()]
        for fraction in s[1:]:
            q = k.ik(previous + fraction*(end-previous), orientation, q)
            qs.append(q.copy())
        phase(name, qs, 8.)
        previous = end
    plan = dict(joint_names=k.names, phases=phases, object_name='box_b',
                expected_object_pose=dict(position=object_target.tolist(), orientation=[0, 0, 0, 1]),
                tcp_orientation=Rotation.from_matrix(orientation).as_quat().tolist(),
                assumptions=['base z=0.002m', 'TCP Link_6 offset [0.0051601583, 0.0014121731, 0.1053659958]m',
                             'ungeometry-derived original +/-3.14 joint limits',
                             'TCP is 0.010m above object center at grasp and place; clears simulator palm box',
                             'arm local mesh AABBs checked against fixed box_a and ground at every sample',
                             'transformed CAD finger bounds and simulator palm box checked; 0.1mm intended closing contact tolerance; self/full mesh collision not certified'])
    validate(plan)
    return plan

def validate(plan):
    k = Kinematics()
    checker = ArmCollisionChecker(k)
    for phase in plan['phases']:
        points = phase['points']
        q = np.array([p['positions'] for p in points])
        times = np.array([p['time_from_start'] for p in points])
        assert np.all(q >= k.lower) and np.all(q <= k.upper)
        assert all(checker.clear(a, [.4, -.25, .025] if phase['name'] in ['pregrasp', 'grasp'] else None) for a in q), phase['name'] + ": arm/box or ground collision"
        assert np.max(np.abs(np.diff(q, axis=0))) < .10
        assert np.all(np.abs(np.diff(q, axis=0) / np.diff(times)[:, None]) <= k.velocity_limits)
        assert np.all(np.abs([p['velocities'] for p in points]) <= k.velocity_limits)
        if phase['name'] != 'pregrasp':
            expected = Rotation.from_quat(plan['tcp_orientation']).as_matrix()
            assert max(np.linalg.norm(k.fk(a)[:3,:3]-expected) for a in q) < 1e-4
    grasp_q = next(p for p in plan['phases'] if p['name'] == 'grasp')['points'][-1]['positions']
    # 0.1 mm contact tolerance only for the intended finger/cube face contact.
    # Check the full palm against the unshrunk cube at the open grasp above.
    for closure in np.linspace(0, .023, 24):
        assert checker.clear(grasp_q, np.array([.4, -.25, .025]), closure) or (
            closure == .023 and checker.clear(grasp_q, np.array([.4, -.25, .025]), .0229))
    for a, b in zip(plan['phases'], plan['phases'][1:]):
        assert np.allclose(a['points'][-1]['positions'], b['points'][0]['positions'])
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'config/pick_place_plan.json')
    args = parser.parse_args()
    plan = generate()
    args.output.parent.mkdir(exist_ok=True)
    args.output.write_text(json.dumps(plan, indent=2) + '\n')
    print('Validated trajectory:', args.output)
