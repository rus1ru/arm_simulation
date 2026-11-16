import importlib.util
from pathlib import Path
import unittest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/run_simulation.py'
SPEC = importlib.util.spec_from_file_location('run_simulation', SCRIPT)
module = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(module)


class RunSimulationTest(unittest.TestCase):
    def test_results_path_with_spaces_is_one_argument(self):
        path = Path('/tmp/Projects with spaces/results.json')
        command = module.pick_place_command(path)
        self.assertEqual(command[-2:], ['--results', str(path)])
        self.assertEqual(len(command), 6)
        self.assertEqual(module.sourced(__import__('shlex').join(command))[-1].count('Projects with spaces'), 1)


if __name__ == '__main__':
    unittest.main()
