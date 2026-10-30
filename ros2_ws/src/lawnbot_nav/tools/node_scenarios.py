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
    print(f"\n{CHECKS['ok']} comprobaciones OK")
