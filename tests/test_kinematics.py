"""Run: /usr/bin/python3 -m unittest discover -s robo2/tests -v"""
import importlib.util
import json
from pathlib import Path
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('gp7', ROOT / 'scripts/gp7_kinematics.py')
gp7 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gp7)

class KinematicsTest(unittest.TestCase):
    def test_saved_plan(self):
        plan = json.loads((ROOT / 'config/pick_place_plan.json').read_text())
        self.assertTrue(gp7.validate(plan))
        k = gp7.Kinematics()
        # TCP stays 10mm above object center at both contact endpoints.
        goals = {'grasp': [.4, -.25, .035], 'place': [.5, .5, .235]}
        for phase in plan['phases']:
            if phase['name'] in goals:
                np.testing.assert_allclose(k.fk(phase['points'][-1]['positions'])[:3, 3],
                                           goals[phase['name']], atol=1e-6)

    def test_fk_ik_roundtrip(self):
        k = gp7.Kinematics()
        q = np.array([.3, -.5, .4, .2, -.7, .1])
        t = k.fk(q)
        actual = k.ik(t[:3,3], t[:3,:3], q + .01)
        np.testing.assert_allclose(k.fk(actual), t, atol=1e-6)

if __name__ == '__main__':
    unittest.main()
