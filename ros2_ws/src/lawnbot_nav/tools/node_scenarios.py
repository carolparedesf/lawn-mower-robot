#!/usr/bin/env python3
"""Escenarios de los nodos de lawnbot_nav contra la capa de ROS falsa (tools/fake_ros.py).

Uso (desde la carpeta del paquete, en una maquina SIN ROS 2):  python3 tools/node_scenarios.py
"""
import json
import math
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))
import fake_ros as R   # noqa: E402

R.install()

from lawnbot_nav.geofence_logic import lat_lon_to_local_xy  # noqa: E402
from lawnbot_nav.sim_robot import SimRobot                   # noqa: E402

CHECKS = {'ok': 0}


def check(cond, text):
    assert cond, f'FALLO: {text}'
    CHECKS['ok'] += 1
    print(f'  ok  {text}')


def make(cls, **params):
    R.NodeBase.PARAM_OVERRIDES = params
    return cls()


def geo_json(points):
    return json.dumps({'points': [{'lat': lat, 'lng': lng} for lat, lng in points]})


LAT0, LNG0 = -25.3000, -57.6000
SQUARE_LL = [(LAT0, LNG0), (LAT0, LNG0 + 0.0002), (LAT0 + 0.0002, LNG0 + 0.0002), (LAT0 + 0.0002, LNG0)]
# ~ 20 m x 22 m
SQUARE_XY = [lat_lon_to_local_xy(la, ln, LAT0, LNG0) for la, ln in SQUARE_LL]


def run_geofence_datum():
    print('\n[1] geofence_node en modo datum')
    from lawnbot_nav.geofence_node import GeofenceNode
    tmp = tempfile.mkdtemp()
    persist = os.path.join(tmp, 'cerco.json')
    g = make(GeofenceNode, conversion_mode='datum', datum_lat=LAT0, datum_lng=LNG0,
             persist_path=persist, hysteresis_in=3, hysteresis_out=2)
    tick, maintain = g.timer(0.1), g.timer(1.0)
    last = lambda name: g.pubs[name].last   # noqa: E731

    def run(n=1):
        for _ in range(n):
            R.advance(0.1)
            tick()
        maintain()

    run()
    check(last('geofence/status').data == 'NO_POLYGON', 'sin cerco: NO_POLYGON')
    check(last('geofence/inside').data is False and math.isnan(last('geofence/distance_to_edge').data),
          'sin cerco: inside=false y distancia NaN')

    g.subs['geofence_raw'](R.String(geo_json(SQUARE_LL)))
    run()
    check(last('geofence/status').data == 'NO_POSE', 'cerco convertido, sin pose: NO_POSE')
    poly = last('geofence/polygon').polygon.points
    check(len(poly) == 4 and last('geofence/polygon').header.frame_id == 'map',
          'polygon publicado en map con 4 vertices')
    check(os.path.exists(persist), 'cerco guardado en disco (lat/lon)')

    R.set_pose('map', 'base_footprint', 10.0, 11.0)
    run(5)
    check(last('geofence/status').data == 'OK', 'con pose: OK')
    check(last('geofence/inside').data is True, 'robot en el centro: inside=true')
    check(abs(last('geofence/distance_to_edge').data - 10.0) < 0.5, 'distancia al borde ~10 m (positiva)')

    R.set_pose('map', 'base_footprint', 40.0, 11.0)
    run(1)
    check(last('geofence/inside').data is True, 'primera lectura fuera: la histeresis sostiene true')
    run(2)
    check(last('geofence/inside').data is False, 'tras 2 lecturas fuera: inside=false')
    check(last('geofence/distance_to_edge').data < 0, 'distancia negativa afuera')

    R.drop_pose('map', 'base_footprint')
    run(1)
    check(last('geofence/inside').data is False and math.isnan(last('geofence/distance_to_edge').data),
          'sin pose: inside=false de inmediato y NaN')
    R.set_pose('map', 'base_footprint', 10.0, 11.0)
    R.advance(0.6)
    g.get_pose()
    stale_stamp = R.CLOCK['ns'] - int(2e9)
    R.set_pose('map', 'base_footprint', 10.0, 11.0, stamp_ns=stale_stamp)
    run(1)
    check(last('geofence/status').data == 'NO_POSE' and last('geofence/inside').data is False,
          'pose vieja (2 s): NO_POSE e inside=false')

    for text, motivo in [(geo_json(SQUARE_LL[:3]), 'menos de 4'),
                         (geo_json([SQUARE_LL[0], SQUARE_LL[2], SQUARE_LL[1], SQUARE_LL[3]]), 'autointerseccion'),
                         ('esto no es json', 'json')]:
        g.subs['geofence_raw'](R.String(text))
        run(1)
        st = last('geofence/status').data
        check(st.startswith('INVALID:') and motivo in st, f'cerco invalido -> {st}')
        check(last('geofence/inside').data is False and len(last('geofence/polygon').polygon.points) == 0,
              '   inside=false y poligono vacio')

    # reinicio del nodo: el cerco se restaura desde el archivo
    g.subs['geofence_raw'](R.String(geo_json(SQUARE_LL)))
    g2 = make(GeofenceNode, conversion_mode='datum', datum_lat=LAT0, datum_lng=LNG0, persist_path=persist)
    check(len(g2.raw) == 4, 'tras reiniciar, el cerco se restaura desde el archivo (lat/lon)')
    g.shutdown_cleanup()
    check(last('geofence/inside').data is False, 'shutdown: inside=false')


def run_geofence_navsat():
    print('\n[2] geofence_node con el servicio fromLL (navsat_transform)')
    from lawnbot_nav.geofence_node import GeofenceNode
    g = make(GeofenceNode, conversion_mode='navsat', persist_path='', hysteresis_in=1, hysteresis_out=1)
    client = g.clients['fromLL']
    tick, maintain = g.timer(0.1), g.timer(1.0)
    last = lambda name: g.pubs[name].last   # noqa: E731

    def answer(offset, zeros=False):
        """Resuelve las llamadas pendientes como lo haria navsat_transform (map = local + offset)."""
        for req, fut in client.calls:
            x, y = lat_lon_to_local_xy(req.ll_point.latitude, req.ll_point.longitude, LAT0, LNG0)
            fut.resolve(R.Msg(map_point=R.Msg(x=0.0 if zeros else x + offset[0], y=0.0 if zeros else y + offset[1])))
        client.calls.clear()

    live = {'pose': (10.0, 11.0)}      # pose del robot; la TF se refresca en cada ciclo

    def run(n=1):
        for _ in range(n):
            R.advance(0.1)
            R.set_pose('map', 'base_footprint', *live['pose'])
            tick()
        maintain()

    R.set_pose('map', 'base_footprint', *live['pose'])
    g.subs['geofence_raw'](R.String(geo_json(SQUARE_LL)))
    run()
    check(last('geofence/status').data == 'WAITING_CONVERSION' and not client.calls,
          'servicio no disponible: WAITING_CONVERSION, sin llamadas')
    check(last('geofence/inside').data is False, '   inside=false mientras espera')

    client.ready = True
    run()
    check(len(client.calls) == 4, 'servicio disponible: pide la conversion de los 4 vertices')
    answer((0.0, 0.0), zeros=True)
    run()
    check(last('geofence/status').data == 'WAITING_CONVERSION',
          'respuesta (0,0) sin transformacion lista: se descarta')
    R.advance(11.0)
    maintain()
    check(len(client.calls) == 4, '   y se reintenta tras reconvert_period_s')
    answer((0.0, 0.0))
    run(2)
    check(last('geofence/status').data == 'OK' and last('geofence/inside').data is True,
          'conversion correcta: OK e inside=true')

    # navsat_transform se reinicia: el origen de map cambia 30 m al este
    client.ready = False
    maintain()
    run(1)
    check(last('geofence/status').data == 'WAITING_CONVERSION' and last('geofence/inside').data is False,
          'servicio caido: el cerco queda sin validez (inside=false)')
    client.ready = True
    maintain()
    answer((-30.0, 0.0))
    run(2)
    poly = last('geofence/polygon').polygon.points
    check(abs(min(p.x for p in poly) - (-30.0)) < 0.5, 'servicio de vuelta: el poligono se reconvirtio al nuevo origen')
    check(last('geofence/inside').data is False, '   el robot (10, 11) ahora queda fuera del cerco movido')
    live['pose'] = (-20.0, 11.0)
    run(2)
    check(last('geofence/inside').data is True, '   y dentro cuando esta en la nueva posicion')

    # el cerco cambia mientras hay una conversion en curso: la respuesta vieja se ignora
    g.subs['geofence_raw'](R.String(geo_json([(la, ln + 0.001) for la, ln in SQUARE_LL])))
    old_calls = list(client.calls)
    g.subs['geofence_raw'](R.String(geo_json(SQUARE_LL)))
    for req, fut in old_calls:
        fut.resolve(R.Msg(map_point=R.Msg(x=999.0, y=999.0)))
    client.calls.clear()
    run(1)
    check(g.poly is None or max(abs(p[0]) for p in g.poly) < 900, 'respuestas de un cerco anterior se ignoran')


def run_follower_auto():
    print('\n[3] waypoint_follower: ruta por topic, auto-inicio y alertas (con fake_robot)')
    import yaml
    from lawnbot_nav.fake_robot import FakeRobot
    from lawnbot_nav.waypoint_follower import WaypointFollower
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), 'config', 'waypoint_follower.yaml')))
    params = cfg['waypoint_follower']['ros__parameters']

    robot_node = make(FakeRobot, frame_id='map', cmd_topic='cmd_vel_waypoint', x=2.0, y=2.0, yaw=0.0, tau=0.1)
    f = make(WaypointFollower, **dict(params, frame_id='map', relative_to_start=False))
    tick, logcb = f.timer(0.05), f.timer(1.0)
    fake_tick = robot_node.timer(0.02)
    cmd_cb = robot_node.subs['cmd_vel_waypoint']
    alerts = f.pubs['nav/alert_in']

    current = {'mode': None, 'last': -1}

    def set_mode(name):
        current['mode'] = name
        current['last'] = int(R.MONO['t'])
        f.subs['nav/active_mode'](R.String(name))

    def step(dt=0.02):
        R.advance(dt)
        if current['mode'] and int(R.MONO['t']) != current['last']:      # mode_manager republica a 1 Hz
            set_mode(current['mode'])
        fake_tick()
        pub = f.pubs['cmd_vel_waypoint']
        n0 = len(pub.sent)
        tick()
        for m in pub.sent[n0:]:
            cmd_cb(m)

    def run(seconds):
        for _ in range(int(seconds / 0.02)):
            step()

    def alerts_since(n):
        return [m.data for m in alerts.sent[n:]]

    run(0.5)
    set_mode('idle')
    f.subs['geofence/status'](R.String('OK'))
    f.subs['geofence/inside'](R.Bool(True))
    path = [(2.0, 2.0), (8.0, 2.0), (8.0, 2.27), (2.0, 2.27)]
    f.subs['follower/path'](R.Path('map', path))
    check(f.core.route == path and f.core.state == 'IDLE', 'ruta recibida: cargada, sigue en IDLE')
    n0 = len(f.pubs['cmd_vel_waypoint'].sent)
    run(1.0)
    check(len(f.pubs['cmd_vel_waypoint'].sent) == n0 and f.core.state == 'IDLE', 'con ruta pero sin modo: no arranca solo')

    f.subs['follower/path'](R.Path('odom', path))
    check(f.get_logger().has('rechazada: frame', 'WARN'), 'ruta en otro frame: rechazada')
    f.subs['follower/path'](R.Path('map', [(0.0, float('nan'))]))
    check(f.get_logger().has('no finitas', 'WARN'), 'ruta con NaN: rechazada')

    # modo auto_parallel con el cerco NO listo: no arranca y avisa
    f.subs['geofence/status'](R.String('WAITING_CONVERSION'))
    a0 = len(alerts.sent)
    set_mode('auto_parallel')
    run(2.5)
    check(f.core.state == 'IDLE', 'auto_parallel con cerco no OK: no arranca')
    check('follower:geofence_not_ready' in alerts_since(a0), 'avisa por /nav/alert_in: geofence_not_ready')

    # el cerco pasa a OK pero el robot esta fuera: sigue sin arrancar
    f.subs['geofence/status'](R.String('OK'))
    f.subs['geofence/inside'](R.Bool(False))
    run(1.5)
    check(f.core.state == 'IDLE', 'cerco OK pero robot fuera: no arranca')
    f.subs['geofence/inside'](R.Bool(True))
    run(0.5)
    check(f.core.state == 'RUNNING', 'cerco OK y robot dentro: arranca solo')
    check(alerts.sent[-1].data == 'follower:none' or 'follower:none' in alerts_since(a0), 'limpia la alerta al arrancar')

    n_cmd = len(f.pubs['cmd_vel_waypoint'].sent)
    run(3)
    check(len(f.pubs['cmd_vel_waypoint'].sent) - n_cmd >= 100 and robot_node.robot.x > 2.5,
          'navega: ordenes a 50 Hz y el robot avanza')
    f.subs['follower/path'](R.Path('map', [(0.0, 0.0), (1.0, 1.0)]))
    check(f.get_logger().has('rechazada: el seguidor esta en RUNNING') or f.core.route == path,
          'ruta nueva en RUNNING: rechazada, se sigue la actual')

    # stop de la app: el modo sale de allowed_modes
    set_mode('stop')
    run(0.2)
    check(f.core.state == 'PAUSED', 'modo stop: PAUSED')
    check(f.pubs['cmd_vel_waypoint'].sent[-1].linear.x == 0.0, '   y publica cero')

    # elegir de nuevo el modo: ruta nueva + auto-inicio
    set_mode('auto_parallel')
    f.subs['follower/path'](R.Path('map', [(robot_node.robot.x, robot_node.robot.y), (9.0, 3.0)]))
    run(1.0)
    check(f.core.state == 'RUNNING' and f.core.route[-1] == (9.0, 3.0), 'volver a elegir el modo: ruta nueva y auto-inicio')
    # ruta vacia en RUNNING: borra y pausa
    f.subs['follower/path'](R.Path('map', []))
    run(0.2)
    check(f.core.state == 'IDLE' and f.core.route is None and f.pubs['cmd_vel_waypoint'].sent[-1].linear.x == 0.0,
          'ruta vacia en RUNNING: IDLE y cero')

    # recorrido completo hasta DONE con el robot simulado
    f.subs['follower/path'](R.Path('map', path))
    set_mode('stop')
    set_mode('auto_parallel')
    robot_node.robot.x, robot_node.robot.y, robot_node.robot.yaw = 2.0, 2.0, 0.0
    t = 0.0
    while f.core.state != 'DONE' and t < 200:
        step()
        t += 0.02
    check(f.core.state == 'DONE', f'ruta completa hasta DONE en {t:.0f} s')
    check(math.dist((robot_node.robot.x, robot_node.robot.y), path[-1]) < 0.2, '   termino en el ultimo punto')
    run(0.5)
    check(f.pubs['cmd_vel_waypoint'].sent[-1].linear.x == 0.0, '   y publica cero al terminar')
    f.shutdown_cleanup()
    check(all(m.linear.x == 0 and m.angular.z == 0 for m in f.pubs['cmd_vel_waypoint'].sent[-5:]),
          'shutdown: ordenes en cero')


def run_gate_alerts():
    print('\n[4] safety_gate: fusion de alertas de /nav/alert_in')
    import yaml
    from lawnbot_nav.safety_gate import SafetyGate
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), 'config', 'safety_gate.yaml')))
    g = make(SafetyGate, **cfg['safety_gate']['ros__parameters'])
    tick = g.timer(0.05)
    alert = lambda: g.pubs['nav/safety_alert'].last.data   # noqa: E731

    sonar = {'left': 1.5, 'center': 1.5, 'right': 1.5}

    def run(seconds):
        for _ in range(int(seconds / 0.05)):
            R.advance(0.05)
            for s, value in sonar.items():
                g.subs[f'sonar/{s}'](R.Range(value))
            for b in ('bumper/B1_front_left', 'bumper/B2_front_right'):
                g.subs[b](R.Bool(False))
            tick()

    run(0.5)
    check(alert() == 'none', 'sin alertas: none')
    g.subs['nav/alert_in'](R.String('follower:geofence_not_ready'))
    run(0.2)
    check(alert() == 'geofence_not_ready', 'alerta del seguidor: se publica')
    sonar['center'] = 0.1
    run(0.1)
    check(alert() == 'obstacle_stop', 'obstaculo cerca tiene prioridad sobre la del seguidor')
    sonar['center'] = 1.5
    run(0.1)
    g.subs['nav/alert_in'](R.String('follower:none'))
    run(3.5)
    check(alert() == 'none', 'follower:none y el vencimiento (3 s) la limpian')
    g.subs['nav/alert_in'](R.String('follower:no_progress'))
    run(2.0)
    check(alert() == 'no_progress', 'alerta vigente dentro del ttl')
    run(2.0)
    check(alert() == 'none', 'si no se renueva, vence a los 3 s')


def run_coverage():
    print('\n[5] coverage_parallel + waypoint_follower + fake_robot (barrido completo en map)')
    import yaml
    from lawnbot_nav.coverage_logic import swath_coverage
    from lawnbot_nav.coverage_parallel import CoverageParallel
    from lawnbot_nav.fake_robot import FakeRobot
    from lawnbot_nav.waypoint_follower import WaypointFollower
    cfg_dir = os.path.join(os.path.dirname(HERE), 'config')
    fol = yaml.safe_load(open(os.path.join(cfg_dir, 'waypoint_follower_map.yaml')))['waypoint_follower']['ros__parameters']
    cov = yaml.safe_load(open(os.path.join(cfg_dir, 'coverage.yaml')))['coverage_parallel']['ros__parameters']

    robot_node = make(FakeRobot, frame_id='map', cmd_topic='cmd_vel_waypoint', x=0.5, y=0.5, yaw=0.0, tau=0.1)
    f = make(WaypointFollower, **fol)
    c = make(CoverageParallel, **cov)
    fake_tick, f_tick = robot_node.timer(0.02), f.timer(0.05)
    c_maintain = c.timer(1.0)
    cmd_cb = robot_node.subs['cmd_vel_waypoint']
    fence_pub = lambda pts: c.subs['geofence/polygon'](_polygon(pts))   # noqa: E731

    state = {'mode': None, 'last': -1, 'forwarded': 0}
    trajectory = []

    def set_mode(name):
        state['mode'], state['last'] = name, int(R.MONO['t'])
        for node in (f, c):
            node.subs['nav/active_mode'](R.String(name))

    def forward_paths():
        """Lo que coverage publica en /follower/path llega al seguidor."""
        sent = c.pubs['follower/path'].sent
        while state['forwarded'] < len(sent):
            f.subs['follower/path'](sent[state['forwarded']])
            state['forwarded'] += 1

    def step(dt=0.02):
        R.advance(dt)
        if state['mode'] and int(R.MONO['t']) != state['last']:
            set_mode(state['mode'])
            c_maintain()                                  # timer de 1 Hz del generador
        fake_tick()
        pub = f.pubs['cmd_vel_waypoint']
        n0 = len(pub.sent)
        f_tick()
        for m in pub.sent[n0:]:
            cmd_cb(m)
        forward_paths()
        trajectory.append((robot_node.robot.x, robot_node.robot.y))

    def run(seconds):
        for _ in range(int(seconds / 0.02)):
            step()

    def status():
        return c.pubs['coverage/status'].last.data

    set_mode('idle')
    f.subs['geofence/status'](R.String('OK'))
    f.subs['geofence/inside'](R.Bool(True))
    run(0.5)
    check(status() == 'IDLE' and len(c.pubs['follower/path'].sent) == 0, 'fuera de auto_parallel no genera nada')

    # cerco no convexo (en L): error claro y ninguna ruta
    fence_pub(L_SHAPE)
    set_mode('auto_parallel')
    run(1.5)
    check(status().startswith('ERROR:') and 'convexo' in status(), f'cerco no convexo -> {status()}')
    check(len(c.pubs['follower/path'].last.poses) == 0, '   publica una ruta vacia (nada que seguir)')
    check(f.core.route is None and f.core.state == 'IDLE', '   el seguidor no recibe ruta ni arranca')
    check(any(m.data == 'coverage:coverage_error' for m in c.pubs['nav/alert_in'].sent), '   avisa por /nav/alert_in: coverage_error')

    # llega un cerco valido mientras se espera: genera y el seguidor arranca solo
    fence_pub(FENCE)
    run(0.5)
    st = status()
    check(st.startswith('OK lineas='), f'cerco convexo -> {st}')
    path = c.pubs['follower/path'].last
    check(path.header.frame_id == 'map' and len(path.poses) > 20, f'ruta en frame map con {len(path.poses)} puntos')
    marks = c.pubs['coverage/markers'].last.markers
    check(marks[0].action == R.Marker.DELETEALL and {m.ns for m in marks[1:]} == {'fence', 'region', 'path', 'waypoints'},
          'markers: cerco, region, ruta y puntos')
    first = (path.poses[0].pose.position.x, path.poses[0].pose.position.y)
    check(min(math.dist(first, p) for p in [(0.85, 1.0), (5.15, 1.0), (0.85, 4.0), (5.15, 4.0)]) < 1e-6,
          f'empieza por la esquina mas cercana al robot: ({first[0]:.2f}, {first[1]:.2f})')
    R.set_pose('map', 'base_footprint', robot_node.robot.x, robot_node.robot.y)
    t = 0.0
    while f.core.state != 'DONE' and t < 900:
        step()
        t += 0.02
        if f.core.state == 'ERROR':
            break
    check(f.core.state == 'DONE', f'el seguidor recorre el barrido completo hasta DONE en {t:.0f} s')
    region = [(1.0, 1.0), (5.0, 1.0), (5.0, 4.0), (1.0, 4.0)]
    ratio = swath_coverage(region, trajectory, cov['cutting_width'] / 2.0)
    print(f'      area cubierta de la region de trabajo: {ratio * 100:.1f} %')
    check(ratio > 0.97, 'cobertura de la region de trabajo > 97 %')
    check(min(signed_distance_xy(p, FENCE) for p in trajectory) > 0.3, '   el robot nunca se acerco al borde del cerco a menos de 0,3 m')

    # salir del modo: ruta vacia, markers borrados, estado IDLE
    set_mode('stop')
    run(0.3)
    check(status() == 'IDLE' and len(c.pubs['follower/path'].last.poses) == 0, 'salir de auto_parallel: IDLE y ruta borrada')
    check(f.core.route is None, '   y el seguidor se queda sin ruta')

    # margen que no deja region
    c2 = make(CoverageParallel, **dict(cov, margin=3.0))
    c2.subs['nav/active_mode'](R.String('idle'))
    c2.subs['geofence/polygon'](_polygon(FENCE))
    c2.subs['nav/active_mode'](R.String('auto_parallel'))
    st = c2.pubs['coverage/status'].last.data
    check(st.startswith('ERROR:region vacia'), f'margen demasiado grande -> {st[:70]}...')

    # sin cerco
    c3 = make(CoverageParallel, **cov)
    c3.subs['nav/active_mode'](R.String('idle'))
    c3.subs['nav/active_mode'](R.String('auto_parallel'))
    check(c3.pubs['coverage/status'].last.data == 'ERROR:sin cerco', 'sin cerco -> ERROR:sin cerco')

    # arrancar el nodo ya dentro de auto_parallel no genera (solo la entrada al modo)
    c4 = make(CoverageParallel, **cov)
    c4.subs['geofence/polygon'](_polygon(FENCE))
    c4.subs['nav/active_mode'](R.String('auto_parallel'))
    check(len(c4.pubs['follower/path'].sent) == 0, 'primer mensaje de modo: solo toma la referencia')
    c.shutdown_cleanup()
    check(len(c.pubs['follower/path'].last.poses) == 0, 'shutdown: borra la ruta')


def run_random_and_perimeter():
    print('\n[6] random_walk y perimeter con fake_robot en un cuarto, pasando por el safety_gate')
    import yaml
    from lawnbot_nav.fake_robot import FakeRobot
    from lawnbot_nav.geofence_logic import signed_distance
    from lawnbot_nav.perimeter import Perimeter
    from lawnbot_nav.random_walk import RandomWalk
    from lawnbot_nav.safety_gate import SafetyGate
    cfg_dir = os.path.join(os.path.dirname(HERE), 'config')
    load = lambda f, k: yaml.safe_load(open(os.path.join(cfg_dir, f)))[k]['ros__parameters']   # noqa: E731
    room, rw_cfg, pm_cfg, gate_cfg = (load('sim_room.yaml', 'fake_robot'), load('random_walk.yaml', 'random_walk'),
                                      load('perimeter.yaml', 'perimeter'), load('safety_gate.yaml', 'safety_gate'))
    fence = [(0.5, 0.5), (9.5, 0.5), (9.5, 7.5), (0.5, 7.5)]

    robot_node = make(FakeRobot, **dict(room, x=3.0, y=2.0))      # lejos de la franja del borde
    rw = make(RandomWalk, **dict(rw_cfg, seed=5, edge_distance=1.0))
    pm = make(Perimeter, **pm_cfg)
    gate = make(SafetyGate, **gate_cfg)
    nodes = (rw, pm)
    fake_tick, sensors_tick = robot_node.timer(0.02), robot_node.timer(0.2)
    rw_tick, pm_tick, gate_tick = rw.timer(0.05), pm.timer(0.05), gate.timer(0.05)
    robot_cmd = robot_node.subs['cmd_vel']
    st = {'mode': None, 'last_mode': -1, 'sent': {}, 'next_sensor': 0.0, 'next_fence': 0.0, 'fence': False,
          'bumper_presses': 0, 'min_gap': 99.0, 'next_ctrl': R.MONO['t']}
    xy = []

    def set_mode(name):
        st['mode'], st['last_mode'] = name, int(R.MONO['t'])
        for n in nodes + (gate,):
            n.subs['nav/active_mode'](R.String(name))

    def new_msgs(pub, key):
        sent = pub.sent
        out = sent[st['sent'].get(key, 0):]
        st['sent'][key] = len(sent)
        return out

    def deliver_sensors():
        for name, pub in robot_node.sonar_pubs.items():
            for m in new_msgs(pub, f'sonar/{name}'):
                for n in (rw, pm, gate):
                    if f'sonar/{name}' in n.subs:
                        n.subs[f'sonar/{name}'](m)
        for name, pub in robot_node.bumper_pubs.items():
            for m in new_msgs(pub, name):
                st['bumper_presses'] += bool(m.data)
                for n in (rw, pm, gate):
                    if name in n.subs:
                        n.subs[name](m)

    def step(dt=0.02):
        R.advance(dt)
        t = R.MONO['t']
        if st['mode'] and int(t) != st['last_mode']:
            set_mode(st['mode'])
        fake_tick()
        if t >= st['next_sensor']:
            sensors_tick()
            deliver_sensors()
            st['next_sensor'] = t + 0.2
        if t >= st['next_fence']:
            r = robot_node.robot
            if st['fence']:
                rw.subs['geofence/status'](R.String('OK'))
                rw.subs['geofence/distance_to_edge'](R.Float32(signed_distance((r.x, r.y), fence)))
            st['next_fence'] = t + 0.1
        if t >= st['next_ctrl']:                         # los tres nodos corren a 20 Hz
            st['next_ctrl'] += 0.05
            rw_tick()
            pm_tick()
            for key, node, topic in (('rw', rw, 'cmd_vel_random'), ('pm', pm, 'cmd_vel_perimeter')):
                for m in new_msgs(node.pubs[topic], key):
                    gate.subs['cmd_vel_mux'](m)         # twist_mux: lo que llega pasa al gate
            pub = gate.pubs['cmd_vel']
            n0 = len(pub.sent)
            gate_tick()
            for m in pub.sent[n0:]:
                robot_cmd(m)
        r = robot_node.robot
        xy.append((r.x, r.y))
        st['min_gap'] = min(st['min_gap'], robot_node.world.distance((r.x, r.y)))

    def run(seconds):
        for _ in range(int(seconds / 0.02)):
            step()

    count = lambda node, topic: len(node.pubs[topic].sent)   # noqa: E731
    set_mode('idle')
    run(2.0)
    check(count(rw, 'cmd_vel_random') == 0 and count(pm, 'cmd_vel_perimeter') == 0,
          'fuera de su modo ninguno de los dos publica nada')

    # random_walk sin cerco: no se mueve y avisa
    set_mode('auto_random')
    run(3.0)
    r = robot_node.robot
    check(math.dist((r.x, r.y), (3.0, 2.0)) < 0.05, 'auto_random sin cerco OK: el robot no se mueve')
    check(any(m.data == 'random:geofence_not_ready' for m in rw.pubs['nav/alert_in'].sent), '   avisa geofence_not_ready por /nav/alert_in')

    st['fence'] = True
    rw.subs['geofence/polygon'](_polygon(fence))
    step()
    ang = rw.interior_angle()
    check(ang is not None and abs(ang - math.pi / 4) < 0.02,
          f'con el cerco y la pose, calcula el giro hacia el centroide: {math.degrees(ang):.0f} grados (esperado 45)')
    n0 = count(rw, 'cmd_vel_random')
    start_bumpers = st['bumper_presses']
    run(300.0)
    published = count(rw, 'cmd_vel_random') - n0
    check(5800 <= published <= 6200, f'con el cerco OK navega: {published} ordenes en 300 s (~20 Hz)')
    cells = {(int(x), int(y)) for x, y in xy}
    check(len(cells) >= 18, f'recorre el cuarto: {len(cells)} celdas de 1 m visitadas')
    check(st['bumper_presses'] == start_bumpers, 'los sonares frenan antes del contacto: ningun bumper presionado')
    check(st['min_gap'] > 0.03, f'nunca toca una pared ni obstaculo (distancia minima {st["min_gap"]:.2f} m)')
    check(min(signed_distance(p, fence) for p in xy) > 0.0, 'nunca sale del cerco')
    check(count(pm, 'cmd_vel_perimeter') == 0, 'perimeter sigue en silencio durante auto_random')
    cmd_sent = [m for m in gate.pubs['cmd_vel'].sent]
    check(max(abs(m.linear.x) for m in cmd_sent) <= 0.5 + 1e-9, 'todo paso por el safety_gate (unico publicador de cmd_vel)')

    # cambio de modo: random se apaga con ceros y perimeter arranca
    set_mode('auto_perimeter')
    run(1.0)
    tail = rw.pubs['cmd_vel_random'].sent[-3:]
    check(all(m.linear.x == 0 and m.angular.z == 0 for m in tail), 'al salir de auto_random publica ceros')
    n_rw = count(rw, 'cmd_vel_random')
    robot_node.robot.x, robot_node.robot.y, robot_node.robot.yaw, robot_node.robot.v, robot_node.robot.w = 2.0, 1.0, 0.0, 0.0, 0.0
    xy.clear()
    st['min_gap'] = 99.0
    start_bumpers = st['bumper_presses']
    run(150.0)
    check(count(rw, 'cmd_vel_random') == n_rw, 'random_walk no publica mas')
    path_len = sum(math.dist(a, b) for a, b in zip(xy, xy[1:]))
    check(path_len > 12.0, f'perimeter avanza siguiendo el borde: {path_len:.1f} m recorridos')
    check(st['min_gap'] > 0.03, f'sin choques (distancia minima {st["min_gap"]:.2f} m)')
    check(st['bumper_presses'] - start_bumpers >= 3, f'modo base por bumper: {st["bumper_presses"] - start_bumpers} contactos con la pared')
    check(pm.pubs['cmd_vel_perimeter'].last is not None and max(abs(m.linear.x) for m in pm.pubs['cmd_vel_perimeter'].sent) <= 0.15 + 1e-9,
          'velocidad lineal de perimeter <= 0,15 m/s')

    set_mode('stop')
    run(1.0)
    tail = pm.pubs['cmd_vel_perimeter'].sent[-3:]
    check(all(m.linear.x == 0 and m.angular.z == 0 for m in tail), 'al salir de auto_perimeter publica ceros')
    rw.shutdown_cleanup()
    pm.shutdown_cleanup()
    check(all(m.linear.x == 0 for m in rw.pubs['cmd_vel_random'].sent[-5:] + pm.pubs['cmd_vel_perimeter'].sent[-5:]),
          'shutdown: ambos publican cero')

    # sonar lateral pedido con un angulo invalido: se ignora y se sigue por bumper (no falla)
    bad = make(Perimeter, **dict(pm_cfg, use_side_sonar=True, side_sonar_angle_deg=0.0))
    check(bad.get_logger().has('se ignora el sonar lateral', 'WARN'), 'use_side_sonar con 0 grados: avisa que se ignora el sonar lateral')
    bad.subs['nav/active_mode'](R.String('auto_perimeter'))
    for s_ in ('left', 'center', 'right'):
        bad.subs[f'sonar/{s_}'](R.Range(1.5))
    for b_ in ('bumper/B1_front_left', 'bumper/B2_front_right'):
        bad.subs[b_](R.Bool(False))
    R.advance(0.1)
    bad.timer(0.05)()
    check(bad.pubs['cmd_vel_perimeter'].last.linear.x == 0.15 and not bad.core.sonar_enabled,
          '   y sigue funcionando por bumper')


def run_gate_perimeter():
    print('\n[7] safety_gate: umbrales por sonar e ignorar los laterales solo en auto_perimeter')
    import yaml
    from lawnbot_nav.safety_gate import SafetyGate
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), 'config', 'safety_gate.yaml')))['safety_gate']['ros__parameters']

    def build(**over):
        g = make(SafetyGate, **dict(cfg, **over))
        tick = g.timer(0.05)
        state = {'left': 1.5, 'center': 1.5, 'right': 1.5, 'front': False, 'mode': None, 'mode_pub': True}

        def run(seconds, publish=('left', 'center', 'right')):
            for _ in range(int(seconds / 0.05)):
                R.advance(0.05)
                for s_ in publish:
                    g.subs[f'sonar/{s_}'](R.Range(state[s_]))
                for b_ in ('bumper/B1_front_left', 'bumper/B2_front_right'):
                    g.subs[b_](R.Bool(state['front']))
                if state['mode'] and state['mode_pub'] and int(R.MONO['t'] * 20) % 20 == 0:
                    g.subs['nav/active_mode'](R.String(state['mode']))
                g.subs['cmd_vel_mux'](_twist(0.2, 0.0))
                tick()
            return g.pubs['cmd_vel'].last.linear.x
        return g, state, run

    g, st, run = build()
    st['left'] = 0.05
    check(run(1.0) == 0.0, 'sin modo: un sonar lateral a 0,05 m frena (todos cuentan)')
    st['mode'] = 'auto_perimeter'
    check(run(1.5) == 0.2, 'auto_perimeter: el lateral a 0,05 m ya no frena')
    st['right'] = 0.05
    check(run(1.0) == 0.2, '   ni el derecho')
    st['center'] = 0.10
    check(run(1.0) == 0.0, '   pero el central a 0,10 m sigue parando')
    st['center'] = 1.5
    st['front'] = True
    check(run(1.0) == 0.0, '   y el bumper frontal tambien')
    st['front'] = False
    check(run(1.0) == 0.2, '   sin esos, el avance pasa entero (nunca se amplifica: 0,2 -> 0,2)')
    check(max(m.linear.x for m in g.pubs['cmd_vel'].sent) <= 0.2 + 1e-12, '   ninguna orden supera la pedida')
    check(run(1.0, publish=('center',)) == 0.2, '   los laterales sin datos no bloquean mientras se ignoran')
    check(run(1.5, publish=('left', 'right')) == 0.0, '   pero el central sin datos si bloquea (falla segura)')

    st['mode'] = 'auto_random'
    run(0.5)
    check(run(1.5) == 0.0, 'auto_random: el lateral a 0,05 m vuelve a frenar')
    st['mode'] = 'auto_perimeter'
    st['mode_pub'] = True
    run(2.0)
    check(run(0.5) == 0.2, 'de vuelta en auto_perimeter se ignoran otra vez')
    st['mode_pub'] = False                                   # el modo deja de renovarse
    R.advance(4.0)
    check(run(1.0) == 0.0, 'sin un modo vigente (3 s) se vuelven a considerar todos los sonares')

    g2, st2, run2 = build(ignore_side_sonars_in_perimeter=False)
    st2['mode'] = 'auto_perimeter'
    st2['left'] = 0.05
    check(run2(2.0) == 0.0, 'con ignore_side_sonars_in_perimeter: false el lateral frena tambien en auto_perimeter')

    # umbrales por sonar: el izquierdo para recien a 0,05 m; el central conserva 0,30 / 0,15
    g3, st3, run3 = build(sonar_left_slow_distance=0.10, sonar_left_stop_distance=0.05)
    st3['left'] = 0.12
    check(run3(1.0) == 0.2, 'umbral propio del sonar izquierdo (0,10 / 0,05): a 0,12 m no frena')
    st3['left'] = 0.07
    v = run3(1.0)
    check(0.0 < v < 0.2, f'   a 0,07 m frena parcialmente ({v:.2f} m/s)')
    st3['left'] = 1.5
    st3['center'] = 0.10
    check(run3(1.0) == 0.0, '   y el central sigue con sus 0,30 / 0,15')


def _twist(v, w):
    t = R.Twist() if hasattr(R, 'Twist') else None
    t.linear.x, t.angular.z = v, w
    return t


def run_blade():
    print('\n[8] blade_controller + coverage_map + fake_robot: /cutter/cmd pasa a 0 en cada condicion (sin motores)')
    import yaml
    from lawnbot_nav.blade_controller import BladeController
    from lawnbot_nav.coverage_map import CoverageMap
    from lawnbot_nav.fake_robot import FakeRobot
    from lawnbot_nav.geofence_logic import signed_distance
    cfg_dir = os.path.join(os.path.dirname(HERE), 'config')
    load = lambda f, k: yaml.safe_load(open(os.path.join(cfg_dir, f)))[k]['ros__parameters']   # noqa: E731
    fence = [(0.0, 0.0), (10.0, 0.0), (10.0, 6.0), (0.0, 6.0)]
    robot_node = make(FakeRobot, frame_id='map', cmd_topic='cmd_vel', x=1.0, y=3.0, yaw=0.0, tau=0.0)
    check(make(CoverageMap).res == 0.5, 'coverage_map: celdas de 0,5 m por defecto')
    small = make(CoverageMap, resolution=0.25)
    check(small.get_logger().has('menor que 0,5 m', 'WARN'), '   y avisa si se configura una celda menor (error del GPS)')
    cm = make(CoverageMap, **load('coverage_map.yaml', 'coverage_map'))
    bl = make(BladeController, **load('blade_controller.yaml', 'blade_controller'))
    fake_tick, cm_tick, bl_tick = robot_node.timer(0.02), cm.timer(0.1), bl.timer(0.1)
    drive = robot_node.subs['cmd_vel']
    S = {'inside': None, 'grass': True, 'imu': (0.0, 0.0, 0.0, 1.0), 'estop': False,
         'bumpers': {n: False for n in ('bumper/B1_front_left', 'bumper/B2_front_right', 'bumper/B3_left',
                                         'bumper/B4_right', 'bumper/B5_back')},
         'speed': 0.0, 'mute': set(), 'next': R.MONO['t'], 'inside_force': None,
         'req_cutter': False, 'req_trimmer': False, 'force_cut': False}
    sent = {'cutter': 0, 'trimmer': 0, 'passes': 0}
    max_cutter = [0.0]
    motor = {'duty': 0.0, 'last': -1e9}                    # modelo de cutting_motors (sin hardware): watchdog de 0,5 s

    def publish_inputs():
        r = robot_node.robot
        inside = (signed_distance((r.x, r.y), fence) >= 0) if S['inside_force'] is None else S['inside_force']
        feeds = {'inside': lambda: bl.subs['geofence/inside'](R.Bool(inside)),
                 'grass': lambda: bl.subs['vision/grass_present'](R.Bool(S['grass'])),
                 'imu': lambda: bl.subs['imu/data'](R.Imu(*S['imu']))}
        for k, fn in feeds.items():
            if k not in S['mute']:
                fn()
        for name, value in S['bumpers'].items():
            if name not in S['mute']:
                bl.subs[name](R.Bool(value))
        bl.subs['estop'](R.Bool(S['estop']))
        # el bridge republica los pedidos a 4 Hz; aca se refrescan en cada ciclo de 10 Hz
        for key, topic in (('req_cutter', 'cutter/enable'), ('req_trimmer', 'trimmer/enable'), ('force_cut', 'blade/force_cut')):
            if key not in S['mute']:
                bl.subs[topic](R.Bool(S[key]))

    def step(dt=0.02):
        R.advance(dt)
        t = R.MONO['t']
        if S['speed']:
            m = R.Twist()
            m.linear.x = S['speed']
            drive(m)
        fake_tick()
        if t >= S['next']:                                  # los nodos y las entradas corren a 10 Hz
            S['next'] += 0.1
            publish_inputs()
            cm_tick()
            for m in cm.pubs['coverage/cell_passes'].sent[sent['passes']:]:
                if 'passes' not in S['mute']:
                    bl.subs['coverage/cell_passes'](m)
            sent['passes'] = len(cm.pubs['coverage/cell_passes'].sent)
            bl_tick()
            for m in bl.pubs['cutter/cmd'].sent[sent['cutter']:]:
                cm.subs['cutter/cmd'](m)
                max_cutter[0] = max(max_cutter[0], m.data)
                motor['duty'], motor['last'] = m.data, t
            sent['cutter'] = len(bl.pubs['cutter/cmd'].sent)

    def run(seconds):
        for _ in range(int(round(seconds / 0.02))):
            step()

    cutter = lambda: bl.pubs['cutter/cmd'].last.data     # noqa: E731
    trimmer = lambda: bl.pubs['trimmer/cmd'].last.data   # noqa: E731
    reasons = lambda m: bl.pubs['blade/status'].last.data   # noqa: E731
    status = lambda: bl.pubs['blade/status'].last.data   # noqa: E731

    cm.subs['geofence/polygon'](_polygon(fence))
    run(2.0)
    check(cutter() == 0.0 and trimmer() == 0.0 and 'sin_pedido' in status(), 'sin pedido: todo en 0 (y se publica el 0 a 10 Hz)')
    S['req_cutter'] = S['req_trimmer'] = True
    run(1.0)
    check(cutter() == 0.3 and trimmer() == 0.3, f'pedido y todo en orden: cuchilla {cutter()} y trimmer {trimmer()} (tope 0,30)')
    check(motor['duty'] == 0.3, '   el modelo de cutting_motors recibe 0,30')

    def trips(text, setup, undo, both=True, expect=None, hold=0.0):
        setup()
        run(0.25)                                            # un par de ciclos de 10 Hz
        ok = cutter() == 0.0 and (not both or trimmer() == 0.0)
        check(ok and (expect is None or expect in status()), f'{text} -> /cutter/cmd = 0' + (' y /trimmer/cmd = 0' if both else '') + f'   [{status()}]')
        undo()
        run(0.3 + hold)
        check(cutter() == 0.3, '   y al resolverse vuelve a encender')

    # ── una por una, las condiciones ──
    def req_off():
        S['req_cutter'] = False
    def req_on():
        S['req_cutter'] = True
    trips('sin pedido de la cuchilla (false del bridge)', req_off, req_on, both=False)

    # el pedido vence a los ~2 s sin refresco (el bridge murio o se perdio el modo)
    S['mute'].add('req_cutter')
    t0 = R.MONO['t']
    run(1.0)
    check(cutter() == 0.3, 'pedido sin refrescar por 1 s: todavia vigente')
    while cutter() != 0.0 and R.MONO['t'] - t0 < 5.0:
        run(0.1)
    waited = R.MONO['t'] - t0
    check(cutter() == 0.0 and 1.9 <= waited <= 2.3 and trimmer() == 0.3,
          f'pedido de la cuchilla sin refresco: vence a los {waited:.1f} s (el del trimmer, refrescado, sigue)')
    S['mute'].discard('req_cutter')
    run(0.3)
    check(cutter() == 0.3, '   y al volver el refresco vuelve a encender')

    # force_cut: anula solo el pasto
    S['grass'] = False
    run(0.3)
    check(cutter() == 0.0, 'sin pasto y sin force_cut: la cuchilla en 0')
    S['force_cut'] = True
    run(0.3)
    check(cutter() == 0.3 and 'force_cut' not in status(), 'con force_cut: la cuchilla corta aunque no haya pasto')
    S['mute'].add('grass')
    run(1.3)
    check(cutter() == 0.3, '   tambien con el dato de pasto viejo')
    S['mute'].discard('grass')
    for text, setup, undo in (
            ('fuera del cerco', lambda: S.update(inside_force=False), lambda: S.update(inside_force=None)),
            ('estop', lambda: S.update(estop=True), lambda: S.update(estop=False)),
            ('volcado', lambda: S.update(imu=(0.4067, 0.0, 0.0, 0.9135)), lambda: S.update(imu=(0.0, 0.0, 0.0, 1.0)))):
        setup()
        run(0.3)
        check(cutter() == 0.0, f'force_cut NO anula: {text}')
        undo()
        run(3.5)
    S['bumpers']['bumper/B3_left'] = S['bumpers']['bumper/B4_right'] = True
    run(0.3)
    check(cutter() == 0.0, 'force_cut NO anula: levantado')
    S['bumpers']['bumper/B3_left'] = S['bumpers']['bumper/B4_right'] = False
    run(3.5)
    S['mute'].add('imu')
    run(0.8)
    check(cutter() == 0.0, 'force_cut NO anula: dato viejo del IMU')
    S['mute'].discard('imu')
    run(0.3)
    S['force_cut'] = False
    S['grass'] = True
    run(0.3)

    # umbral de volcado por defecto: 20 grados
    S['imu'] = (0.1305, 0.0, 0.0, 0.9914)                    # roll de 15 grados
    run(0.3)
    check(cutter() == 0.3, 'inclinacion de 15 grados: sigue cortando')
    S['imu'] = (0.2164, 0.0, 0.0, 0.9763)                    # roll de 25 grados
    run(0.25)
    check(cutter() == 0.0 and 'volcado' in status(), 'inclinacion de 25 grados (> 20): apaga en el ciclo siguiente')
    S['imu'] = (0.0, 0.0, 0.0, 1.0)
    run(2.5)
    check(cutter() == 0.0, '   ya nivelada sigue apagada durante la espera')
    run(1.0)
    check(cutter() == 0.3, '   y rearma a los 3 s')
    def inside_off():
        S['inside_force'] = False
    def inside_on():
        S['inside_force'] = None
    trips('fuera del cerco', inside_off, inside_on, expect='fuera_del_cerco')
    def grass_off():
        S['grass'] = False
    def grass_on():
        S['grass'] = True
    S['grass'] = False
    run(0.25)
    check(cutter() == 0.0 and trimmer() == 0.3, 'sin pasto: la cuchilla en 0 pero el trimmer sigue (el pasto solo afecta a la cuchilla)')
    S['grass'] = True
    run(0.3)
    for a, b in (('bumper/B3_left', 'bumper/B4_right'), ('bumper/B1_front_left', 'bumper/B5_back'), ('bumper/B2_front_right', 'bumper/B5_back')):
        def lift(a=a, b=b):
            S['bumpers'][a] = S['bumpers'][b] = True
        def drop(a=a, b=b):
            S['bumpers'][a] = S['bumpers'][b] = False
        trips(f'levantado ({a[7:9]} + {b[7:9]})', lift, drop, expect='levantado', hold=3.0)
    S['bumpers']['bumper/B1_front_left'] = True
    S['bumpers']['bumper/B3_left'] = True
    run(0.3)
    check(cutter() == 0.3, 'dos bumpers que no son un par de levantado (B1 + B3): sigue cortando')
    S['bumpers']['bumper/B1_front_left'] = S['bumpers']['bumper/B3_left'] = False
    def tilt():
        S['imu'] = (0.4067, 0.0, 0.0, 0.9135)               # roll de 48 grados
    def level():
        S['imu'] = (0.0, 0.0, 0.0, 1.0)
    trips('volcado (roll de 48 grados)', tilt, level, expect='volcado', hold=3.0)
    def estop_on():
        S['estop'] = True
    def estop_off():
        S['estop'] = False
    trips('estop', estop_on, estop_off, expect='estop')

    # ── datos viejos: cada entrada requerida que deja de llegar ──
    for key, what in (('inside', '/geofence/inside'), ('grass', '/vision/grass_present'), ('imu', '/imu/data'),
                      ('bumper/B5_back', 'un bumper'), ('passes', '/coverage/cell_passes')):
        S['mute'].add(key)
        t0 = R.MONO['t']
        while cutter() != 0.0 and R.MONO['t'] - t0 < 3.0:
            run(0.1)
        waited = R.MONO['t'] - t0
        both = key != 'grass'
        check(cutter() == 0.0 and (trimmer() == 0.0 or not both) and waited <= 1.3,
              f'dato viejo ({what}): /cutter/cmd = 0 a los {waited:.1f} s [{status()}]')
        S['mute'].discard(key)
        run(0.6)
        check(cutter() == 0.3, '   y vuelve al reanudarse el dato')

    # ── pasadas por celda: la tercera pasada por la misma linea ya no corta ──
    R.advance(0.0)
    robot_node.robot.x, robot_node.robot.y = 1.0, 3.0
    run(0.5)
    outcomes, third_status = [], set()
    for k in range(3):
        S['speed'] = 0.5 if k % 2 == 0 else -0.5
        start = R.MONO['t']
        while (robot_node.robot.x < 8.0 if k % 2 == 0 else robot_node.robot.x > 2.0) and R.MONO['t'] - start < 30:
            run(0.1)
            if 4.0 < robot_node.robot.x < 6.0:
                outcomes.append((k, cutter()))
                if k == 2:
                    third_status.add(status())
        S['speed'] = 0.0
        run(0.5)
    by_pass = {k: {c for kk, c in outcomes if kk == k} for k in range(3)}
    check(by_pass[0] == {0.3} and by_pass[1] == {0.3}, 'primera y segunda pasada por la linea: corta (celdas con 0 y 1 pasadas)')
    check(by_pass[2] == {0.0} and all('celda_con_demasiadas_pasadas' in t for t in third_status),
          f'tercera pasada: no corta (2 pasadas completadas) [{sorted(third_status)[0]}]')
    check(max_cutter[0] <= 0.30 + 1e-12, 'en todo el escenario ningun duty supero el tope de 0,30')

    # ── el modelo del motor: el watchdog de 0,5 s ya lo apagaria, pero no se depende de el ──
    bl.shutdown_cleanup()
    check(all(m.data == 0.0 for m in bl.pubs['cutter/cmd'].sent[-5:] + bl.pubs['trimmer/cmd'].sent[-5:]), 'shutdown: ambos motores en 0')
    cm.shutdown_cleanup()
    check(cm.pubs['coverage/cell_passes'].last.data == -1, 'shutdown de coverage_map: pasadas desconocidas (-1)')


def run_bridge_actuators():
    print('\n[9] bridge: robot/cmd/actuator -> /cutter/enable, /trimmer/enable, /blade/force_cut  (+ vision_stub)')
    import importlib.util
    import threading
    import types as _types
    repo = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))

    class FakeMqtt:
        class CallbackAPIVersion:
            VERSION2 = 2

        class Client:
            def __init__(self, *a, **k):
                self.published, self.subscribed = [], []
                self.on_connect = self.on_message = None

            def connect(self, *a, **k):
                pass

            def loop_forever(self):
                pass

            def subscribe(self, topic, qos=0):
                self.subscribed.append((topic, qos))

            def publish(self, topic, payload, qos=0, retain=False):
                self.published.append((topic, payload, qos, retain))

    paho = _types.ModuleType('paho')
    paho_mqtt = _types.ModuleType('paho.mqtt')
    paho_client = _types.ModuleType('paho.mqtt.client')
    paho_client.Client, paho_client.CallbackAPIVersion = FakeMqtt.Client, FakeMqtt.CallbackAPIVersion
    paho_mqtt.client = paho_client
    sys.modules.update({'paho': paho, 'paho.mqtt': paho_mqtt, 'paho.mqtt.client': paho_client,
                        'gpsd': _types.ModuleType('gpsd')})
    real_thread = threading.Thread

    class NoThread:
        def __init__(self, *a, **k):
            pass

        def start(self):
            pass
    threading.Thread = NoThread
    try:
        spec = importlib.util.spec_from_file_location('bridge_under_test', os.path.join(repo, 'bridge', 'bridge.py'))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        b = make(mod.RobotBridge)
    finally:
        threading.Thread = real_thread

    by_name = lambda name: [cb for _, cb in b.timers if getattr(cb, '__name__', '') == name][0]   # noqa: E731
    enable_tick, status_tick = by_name('publish_enables'), by_name('publish_status')
    pubs = {'cutter': b.pubs['cutter/enable'], 'trimmer': b.pubs['trimmer/enable'], 'force': b.pubs['blade/force_cut']}

    def mqtt_msg(topic, payload):
        raw = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
        b.on_message(None, None, R.Msg(topic=topic, payload=raw))

    def current():
        return tuple(bool(p.last.data) for p in pubs.values())

    beat = {'mode': None, 'on': True, 'n': 0}

    def tick(seconds):
        for _ in range(int(round(seconds / 0.25))):
            R.advance(0.25)
            beat['n'] += 1
            if beat['on'] and beat['mode'] and beat['n'] % 4 == 0:        # mode_manager republica /nav/active_mode a 1 Hz
                b.subs['nav/active_mode'](R.String(beat['mode']))
            enable_tick()

    def mode(name):
        beat['mode'] = name
        b.subs['nav/active_mode'](R.String(name))

    def status():
        status_tick()
        return json.loads(b.mqtt.published[-1][1])

    check(any(t == 'robot/cmd/actuator' for t, _ in b.mqtt.subscribed) is False, 'antes de conectar no esta suscripto (on_connect aun no corrio)')
    b.on_connect(b.mqtt, None, None, 0, None)
    check(('robot/cmd/actuator', 1) in b.mqtt.subscribed, 'al conectar se suscribe a robot/cmd/actuator con QoS 1')
    tick(1.0)
    check(current() == (False, False, False), 'arranca siempre apagado')

    mode('idle')
    mqtt_msg('robot/cmd/actuator', {'blade': True, 'trimmer': True, 'charging': False})
    tick(1.0)
    check(current() == (False, False, False), 'pedido en modo idle: se ignora, todo apagado')
    st = status()
    check(st['blade'] is False and st['trimmer'] is False and st['mode'] == 'idle', '   y robot/status informa blade=false (el switch de la app no se enciende)')

    mode('auto_parallel')
    mqtt_msg('robot/cmd/actuator', {'blade': True, 'trimmer': False, 'charging': False})
    n0 = len(pubs['cutter'].sent)
    tick(0.5)
    check(current() == (True, False, False), 'payload real de la app en modo auto: /cutter/enable true, /trimmer/enable false')
    tick(9.5)
    count = len(pubs['cutter'].sent) - n0
    check(36 <= count <= 44, f'se republica a 4 Hz mientras se mantenga: {count} mensajes en 10 s')
    st = status()
    check(st['blade'] is True and st['trimmer'] is False and 'alert' in st and 'mode' in st, 'robot/status lleva blade/trimmer pedidos y conserva mode y alert')
    b.on_motor_cmd('cutter', R.Float32(0.3))
    check(status()['blade_running'] is True, '   y blade_running cuando el motor corre de verdad (/cutter/cmd > 0)')
    R.advance(1.5)
    check(status()['blade_running'] is False, '   y deja de estarlo si /cutter/cmd se vuelve viejo')

    mqtt_msg('robot/cmd/actuator', {'blade': True, 'trimmer': True, 'charging': False, 'force_cut': True})
    tick(0.5)
    check(current() == (True, True, True), 'force_cut opcional del payload -> /blade/force_cut true')
    mqtt_msg('robot/cmd/actuator', {'blade': True, 'trimmer': True, 'charging': False})
    tick(0.5)
    check(current() == (True, True, False), '   y sin force_cut en el payload vuelve a false')

    mqtt_msg('robot/cmd/actuator', {'blade': False, 'trimmer': False, 'charging': False})
    tick(0.5)
    check(current() == (False, False, False), 'emergencyStop de la app (todo false): apaga')

    for new_mode in ('idle', 'stop', 'manual'):
        mode('auto_random')
        mqtt_msg('robot/cmd/actuator', {'blade': True, 'trimmer': True})
        tick(0.5)
        mode(new_mode)
        tick(0.5)
        check(current() == (False, False, False), f'al pasar a {new_mode}: se apaga')
        mode('auto_random')
        tick(1.0)
        check(current() == (False, False, False), '   y al volver al modo auto sigue apagado hasta que la app pida de nuevo')

    mode('auto_perimeter')
    mqtt_msg('robot/cmd/actuator', {'blade': True, 'trimmer': True})
    tick(0.5)
    check(current() == (True, True, False), 'pedido de nuevo en auto_perimeter: enciende')
    beat['on'] = False                                       # nadie renueva /nav/active_mode: heartbeat del modo perdido
    tick(4.0)
    check(current() == (False, False, False), 'se pierde el heartbeat del modo (3 s sin /nav/active_mode): se apaga')
    beat['on'] = True
    mode('auto_perimeter')
    tick(1.0)
    check(current() == (False, False, False), '   y no vuelve solo cuando el modo reaparece')

    mode('auto_parallel')
    for junk in (b'no es json', json.dumps([1, 2]).encode(), json.dumps({'blade': 'true', 'trimmer': 1}).encode()):
        mqtt_msg('robot/cmd/actuator', junk)
        tick(0.5)
        check(current() == (False, False, False), f'payload invalido {junk[:20]!r}: no enciende nada')

    # los topics existentes del contrato siguen igual
    mqtt_msg('robot/cmd/mode', {'mode': 'manual'})
    check(b.pubs['robot_mode'].last.data == 'manual', 'robot/cmd/mode sigue yendo a /robot_mode')
    b.subs['coverage/map'](R.String('{"x": 1}'))
    check(b.mqtt.published[-1] == ('robot/map', '{"x": 1}', 0, True), 'robot/map: el mapa de cobertura se reenvia retenido')
    b.relay.clear()

    # ── /robot/events -> robot/status.alert ──
    last_alert = lambda: json.loads([m for m in b.mqtt.published if m[0] == 'robot/status'][-1][1])['alert']   # noqa: E731
    b.subs['robot/events'](R.String('low_battery'))
    check(last_alert() == 'low_battery', 'robot/events: el evento llega a la app como alert')
    b.subs['nav/safety_alert'](R.String('geofence_breach'))
    status_tick()
    check(last_alert() == 'geofence_breach', '   la alerta de seguridad manda sobre el evento')
    b.subs['nav/safety_alert'](R.String('none'))
    R.advance(6.0)
    status_tick()
    check(last_alert() == 'none', '   y el evento vence a los 5 s')

    # ── vision_stub: grass_present true solo con el parametro explicito y con advertencia ──
    from lawnbot_nav.vision_stub import VisionStub
    v = make(VisionStub)
    v.timer(0.2)()
    check(v.pubs['vision/grass_present'].last.data is False and not v.get_logger().has('NO hay deteccion', 'WARN'),
          'vision_stub por defecto: grass_present false y sin advertencia')
    v2 = make(VisionStub, grass_present=True)
    check(v2.get_logger().has('NO hay deteccion real de pasto', 'WARN'), 'con grass_present true: advertencia al arrancar')
    tick2 = v2.timer(0.2)
    n_warn = sum(1 for lv, t in v2.get_logger().lines if lv == 'WARN')
    R.advance(11.0)
    tick2()
    check(v2.pubs['vision/grass_present'].last.data is True and sum(1 for lv, t in v2.get_logger().lines if lv == 'WARN') == n_warn + 1,
          '   y la repite cada 10 s mientras este activo')
    v2.shutdown_cleanup()
    check(v2.pubs['vision/grass_present'].last.data is False, 'shutdown: grass_present vuelve a false')


def run_events():
    print('\n[10] robot_events: eventos por flanco hacia /robot/events (sin tocar el firmware)')
    import yaml
    from lawnbot_nav.robot_events import RobotEvents
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(HERE), 'config', 'robot_events.yaml')))['robot_events']['ros__parameters']
    n = make(RobotEvents, **cfg)
    tick = n.timer(0.1)
    sent = lambda: [m.data for m in n.pubs['robot/events'].sent]   # noqa: E731
    B = ('bumper/B1_front_left', 'bumper/B2_front_right', 'bumper/B3_left', 'bumper/B4_right', 'bumper/B5_back')
    S = {'mode': 'idle', 'cutter': 0.0, 'bat': 0.80, 'bumpers': set()}

    def run(seconds):
        for _ in range(int(seconds * 10)):
            R.advance(0.1)
            n.subs['nav/active_mode'](R.String(S['mode']))
            n.subs['cutter/cmd'](R.Float32(S['cutter']))
            n.subs['trimmer/cmd'](R.Float32(0.0))
            for b in B:
                n.subs[b](R.Bool(b in S['bumpers']))
            bs = R.BatteryState()
            bs.percentage, bs.voltage = S['bat'], 24.0
            n.subs['battery/state'](bs)
            tick()

    run(1.0)
    check(sent() == [], 'arranque en idle: sin eventos')
    S['mode'] = 'auto_perimeter'
    run(2.0)
    check(sent() == ['perimeter_started'], 'auto_perimeter -> perimeter_started una vez')
    S['mode'] = 'auto_parallel'
    run(2.0)
    check(sent()[-1] == 'navigation_started' and len(sent()) == 2, 'auto_parallel -> navigation_started')
    S['cutter'] = 0.3
    run(2.0)
    check(sent()[-1] == 'cutting_started' and len(sent()) == 3, '/cutter/cmd > 0 -> cutting_started')
    S['bumpers'] = {'bumper/B3_left', 'bumper/B4_right'}
    run(1.0)
    S['bumpers'] = set()
    run(5.0)
    check(sent().count('lifted') == 1, 'B3+B4 a la vez -> lifted una sola vez')
    S['bat'] = 0.15
    run(60.0)
    check(sent().count('low_battery') == 1, 'bateria 15 %: low_battery una sola vez en 60 s')
    S['bat'] = 0.22
    run(5.0)
    S['bat'] = 0.15
    run(5.0)
    check(sent().count('low_battery') == 1, '   dentro de la banda de histeresis no se repite')
    S['bat'] = 0.30
    run(2.0)
    S['bat'] = 0.15
    run(5.0)
    check(sent().count('low_battery') == 2, '   rearma al pasar umbral + margen')
    n_before = len(sent())
    n.mode_time -= 10.0
    S['mode'] = 'idle'
    run(3.0)
    check(len(sent()) == n_before, 'volver a idle no genera eventos')


FENCE = [(0.0, 0.0), (6.0, 0.0), (6.0, 5.0), (0.0, 5.0)]
L_SHAPE = [(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (4.0, 4.0), (4.0, 10.0), (0.0, 10.0)]


def _polygon(points, frame='map'):
    poly = R.PolygonStamped()
    poly.header.frame_id = frame
    poly.polygon.points = [R.Point32(x, y, 0.0) for x, y in points]
    return poly


def signed_distance_xy(pt, poly):
    from lawnbot_nav.geofence_logic import signed_distance
    return signed_distance(pt, poly)


if __name__ == '__main__':
    run_geofence_datum()
    run_geofence_navsat()
    run_follower_auto()
    run_gate_alerts()
    run_coverage()
    run_random_and_perimeter()
    run_gate_perimeter()
    run_blade()
    run_bridge_actuators()
    run_events()
    print(f"\n{CHECKS['ok']} comprobaciones OK")
