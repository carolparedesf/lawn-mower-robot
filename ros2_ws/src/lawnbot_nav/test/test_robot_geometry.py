"""El angulo de los sonares de las esquinas sale de UN parametro (config/robot_geometry.yaml): el URDF generado no se desfasa."""
import math
import os
import unittest
import xml.etree.ElementTree as ET

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, '..', 'config')
URDF_DIR = os.path.join(HERE, '..', '..', '..', '..', 'mow_simplified', 'urdf')


def shared_angle():
    with open(os.path.join(CONFIG, 'robot_geometry.yaml')) as f:
        return yaml.safe_load(f)['/**']['ros__parameters']['corner_sonar_angle_deg']


def shared(name):
    with open(os.path.join(CONFIG, 'robot_geometry.yaml')) as f:
        return yaml.safe_load(f)['/**']['ros__parameters'][name]


def urdf_xyz(path, joint):
    for j in ET.parse(path).getroot().iter('joint'):
        if j.get('name') == joint:
            return [float(v) for v in j.find('origin').get('xyz').split()]
    raise KeyError(joint)


def urdf_yaw(path, joint):
    for j in ET.parse(path).getroot().iter('joint'):
        if j.get('name') == joint:
            return float(j.find('origin').get('rpy').split()[2])
    raise KeyError(joint)


class AnguloCompartidoTests(unittest.TestCase):
    def test_el_urdf_generado_usa_el_angulo_del_yaml(self):
        angle = math.radians(shared_angle())
        path = os.path.join(URDF_DIR, 'mow_simplified.urdf')
        self.assertAlmostEqual(urdf_yaw(path, 'base_sonarL_joint'), angle, places=6)
        self.assertAlmostEqual(urdf_yaw(path, 'base_sonarR_joint'), -angle, places=6)
        self.assertEqual(urdf_yaw(path, 'base_sonarC_joint'), 0.0)

    def test_las_ruedas_estan_a_mas_menos_la_mitad_de_la_separacion(self):
        sep = shared('wheel_separation')
        self.assertEqual(sep, 0.355)                                  # igual que base_width del firmware
        path = os.path.join(URDF_DIR, 'mow_simplified.urdf')
        self.assertAlmostEqual(urdf_xyz(path, 'base_BLwheel_joint')[1], sep / 2.0, places=9)
        self.assertAlmostEqual(urdf_xyz(path, 'base_BRwheel_joint')[1], -sep / 2.0, places=9)
        self.assertAlmostEqual(urdf_xyz(path, 'base_BLwheel_joint')[1] - urdf_xyz(path, 'base_BRwheel_joint')[1], 0.355, places=9)

    def test_la_separacion_del_simulador_es_la_misma(self):
        import inspect
        from lawnbot_nav.sim_robot import SimRobot
        self.assertEqual(inspect.signature(SimRobot.__init__).parameters['base_width'].default, shared('wheel_separation'))

    def test_el_xacro_lee_el_mismo_yaml_sin_valores_propios(self):
        with open(os.path.join(URDF_DIR, 'mow_simplified.urdf.xacro')) as f:
            text = f.read()
        self.assertIn('geometry_file', text)
        self.assertIn('robot_geometry.yaml', text)
        self.assertIn('${corner_sonar_yaw}', text)
        self.assertIn('${-corner_sonar_yaw}', text)
        self.assertIn('${wheel_half_separation}', text)
        self.assertIn('${-wheel_half_separation}', text)
        self.assertNotIn('0.17932', text)                           # sin la separacion vieja (0,36 m) en la fuente
        self.assertNotIn('-0.18068', text)

    def test_los_launch_cargan_el_archivo_compartido(self):
        for launch, expected in (('nav.launch.py', 3), ('sim_world.launch.py', 1)):
            with open(os.path.join(HERE, '..', 'launch', launch)) as f:
                self.assertEqual(f.read().count('robot_geometry.yaml'), expected, launch)

    def test_el_yaml_compartido_usa_comodin_de_nodos(self):
        with open(os.path.join(CONFIG, 'robot_geometry.yaml')) as f:
            data = yaml.safe_load(f)
        self.assertEqual(list(data), ['/**'])
        self.assertEqual(list(data['/**']['ros__parameters']), ['corner_sonar_angle_deg', 'wheel_separation'])
        self.assertGreaterEqual(shared_angle(), 20.0)               # el sonar lateral de perimeter exige >= 20 grados


if __name__ == '__main__':
    unittest.main()
