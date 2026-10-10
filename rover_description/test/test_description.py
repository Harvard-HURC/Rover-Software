"""The rover's description: what xacro makes of urdf/rover.urdf.xacro and config/rover.yaml."""
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest
import xml.etree.ElementTree as ET

import xacro
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / 'urdf' / 'rover.urdf.xacro'
ROVER_YAML = PACKAGE / 'config' / 'rover.yaml'
ROVER = yaml.safe_load(ROVER_YAML.read_text())
WHEELS = ('wheel_fl', 'wheel_rl', 'wheel_fr', 'wheel_rr')


def expand(**args):
    """Return the description expanded with these xacro arguments, as text."""
    doc = xacro.process_file(str(URDF), mappings={k: str(v) for k, v in args.items()})
    return doc.toprettyxml(indent='  ')


def robot(**args):
    """Return the expanded description's <robot> element."""
    return ET.fromstring(expand(**args))


def numbers(text):
    """Return the numbers of a URDF attribute such as xyz or size."""
    return [float(v) for v in text.split()]


def load_module(path, name):
    """Import a Python file that is not on the path (a script, a launch file)."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Description(unittest.TestCase):

    def test_check_urdf_accepts_it(self):
        if shutil.which('check_urdf') is None:
            self.skipTest('check_urdf is not installed (liburdfdom-tools)')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'rover.urdf'
            path.write_text(expand())
            result = subprocess.run(['check_urdf', str(path)], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('Successfully Parsed XML', result.stdout)
        self.assertIn('root Link: base_link', result.stdout)

    def test_joint_tree(self):
        joints = {j.get('name'): (j.get('type'), j.find('parent').get('link'),
                                  j.find('child').get('link'))
                  for j in robot().findall('joint')}
        self.assertEqual(joints, {
            'imu_joint': ('fixed', 'base_link', 'imu'),
            'gnss_joint': ('fixed', 'base_link', 'gnss'),
            'led_joint': ('fixed', 'base_link', 'led'),
            'camera_pan_joint': ('revolute', 'base_link', 'camera_pan_link'),
            'camera_tilt_joint': ('revolute', 'camera_pan_link', 'camera_tilt_link'),
            'camera_joint': ('fixed', 'camera_tilt_link', 'camera'),
            'rocker_left_joint': ('revolute', 'base_link', 'rocker_left'),
            'wheel_fl_joint': ('continuous', 'rocker_left', 'wheel_fl'),
            'wheel_rl_joint': ('continuous', 'rocker_left', 'wheel_rl'),
            'rocker_right_joint': ('revolute', 'base_link', 'rocker_right'),
            'wheel_fr_joint': ('continuous', 'rocker_right', 'wheel_fr'),
            'wheel_rr_joint': ('continuous', 'rocker_right', 'wheel_rr'),
        })

    def test_wheels_sit_where_rover_yaml_says(self):
        rocker, wheel = ROVER['rocker'], ROVER['wheel']
        joints = {j.get('name'): j for j in robot().findall('joint')}
        for side, sign in (('left', 1), ('right', -1)):
            pivot = numbers(joints[f'rocker_{side}_joint'].find('origin').get('xyz'))
            self.assertEqual(pivot, [0.0, sign * rocker['pivot_y'], rocker['pivot_z']])
            for end, ahead in (('f', 1), ('r', -1)):
                offset = numbers(joints[f'wheel_{end}{side[0]}_joint'].find('origin').get('xyz'))
                self.assertEqual(offset, [ahead * wheel['dx'], 0.0, wheel['dz']])
        self.assertAlmostEqual(rocker['pivot_z'] + wheel['dz'], wheel['radius'])  # on the ground

    def test_a_changed_rover_yaml_changes_the_rover(self):
        changed = yaml.safe_load(ROVER_YAML.read_text())
        changed['wheel']['radius'] = 0.2
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'rover.yaml'
            path.write_text(yaml.safe_dump(changed))
            link = robot(rover_yaml=path).find("link[@name='wheel_fl']")
        self.assertEqual(float(link.find('collision/geometry/cylinder').get('radius')), 0.2)
        self.assertEqual(numbers(link.find('visual/geometry/mesh').get('scale')),
                         [0.2, ROVER['wheel']['width'], 0.2])

    def test_inertias_come_from_the_shapes(self):
        description = robot()
        links = {link.get('name'): link for link in description.findall('link')}
        chassis, wheel, head = ROVER['chassis'], ROVER['wheel'], ROVER['head']
        x, y, z = chassis['size']
        inertia = links['base_link'].find('inertial/inertia')
        self.assertAlmostEqual(float(inertia.get('ixx')), chassis['mass'] * (y * y + z * z) / 12)
        self.assertAlmostEqual(float(inertia.get('izz')), chassis['mass'] * (x * x + y * y) / 12)
        iyy = float(links['wheel_fl'].find('inertial/inertia').get('iyy'))
        self.assertAlmostEqual(iyy, wheel['mass'] * wheel['radius'] ** 2 / 2)
        total = sum(float(m.get('value')) for m in description.iter('mass'))
        self.assertAlmostEqual(total, chassis['mass'] + 2 * ROVER['rocker']['mass'] +
                               4 * wheel['mass'] + head['pan_mass'] + head['tilt_mass'])

    def test_it_names_no_simulator(self):
        description = robot()
        self.assertIsNone(description.find('.//gazebo'))
        hardware = set(description.findall('ros2_control/hardware/plugin'))
        self.assertEqual([p for p in description.iter('plugin') if p not in hardware], [])

    def test_ros2_control_block(self):
        block = robot().find('ros2_control')
        self.assertEqual(block.get('type'), 'system')
        self.assertEqual(block.findtext('hardware/plugin'), 'rover_control/RoverHardwareStub')
        interfaces = {j.get('name'): ([c.get('name') for c in j.findall('command_interface')],
                                      [s.get('name') for s in j.findall('state_interface')])
                      for j in block.findall('joint')}
        driven = (['velocity'], ['position', 'velocity', 'effort'])
        sensed = ([], ['position', 'velocity'])
        self.assertEqual(interfaces, {
            **{f'{w}_joint': driven for w in WHEELS},
            **{j: sensed for j in ('rocker_left_joint', 'rocker_right_joint', 'camera_pan_joint',
                                   'camera_tilt_joint')}})
        other = robot(ros2_control_plugin='rover_sim/RoverSimSystem')
        self.assertEqual(other.findtext('ros2_control/hardware/plugin'),
                         'rover_sim/RoverSimSystem')
        self.assertIsNone(robot(ros2_control='false').find('ros2_control'))

    def test_wheel_mesh_is_what_its_generator_writes(self):
        generator = load_module(PACKAGE / 'meshes' / 'gen_wheel_mesh.py', 'gen_wheel_mesh')
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'wheel.glb'
            generator.write_glb(path, generator.wheel_parts(*generator.colors()))
            self.assertEqual(path.read_bytes(), (PACKAGE / 'meshes' / 'wheel.glb').read_bytes())


if __name__ == '__main__':
    unittest.main()
