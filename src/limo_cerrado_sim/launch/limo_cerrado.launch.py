"""LIMO Pro + Livox MID-360 no mundo Cerrado 32x32 m (Gazebo Harmonic, ROS 2 Jazzy).

  ros2 launch limo_cerrado_sim limo_cerrado.launch.py                 # esteiras, 15 arvores, GUI
  ros2 launch limo_cerrado_sim limo_cerrado.launch.py mode:=diff      # quatro rodas (modelo oficial)
  ros2 launch limo_cerrado_sim limo_cerrado.launch.py trees:=68
  ros2 launch limo_cerrado_sim limo_cerrado.launch.py x:=0 y:=0 yaw:=1.57
  ros2 launch limo_cerrado_sim limo_cerrado.launch.py gui:=false rviz:=true

O z do spawn e calculado a partir do mesh do terreno (z:=auto), e o robo nasce ja
alinhado a inclinacao local, entao qualquer x/y dentro de [-16, 16] funciona.
"""

import math
import os
import socket
import struct

import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PKG = 'limo_cerrado_sim'
WORLD_NAME = 'cerrado'
ROBOT_NAME = 'limo'
# variante -> (arquivo do mundo, nome do modelo)
VARIANTS = {
    '15': ('cerrado_15.sdf', 'cerrado_32x32_15_arvores'),
    '68': ('cerrado_68.sdf', 'cerrado_32x32'),
}
# o modelo de 68 arvores (~500 MB) nao e copiado para o pacote; vem do diretorio original
CERRADO68_MODELS_DEFAULT = '/root/cerrado_32x32/models'
SPAWN_CLEARANCE = 0.06  # [m] solta o robo um pouco acima do solo


def terrain_pose(stl_path, x, y):
    """Altura e normal do terreno em (x, y), lidas do STL binario de colisao."""
    with open(stl_path, 'rb') as f:
        f.seek(80)
        n = struct.unpack('<I', f.read(4))[0]
        rec = np.frombuffer(f.read(n * 50), dtype=np.dtype(
            [('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    tri = rec['v'].reshape(-1, 3, 3).astype(np.float64)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    # coordenadas baricentricas de (x, y) em cada triangulo, no plano XY
    v0, v1 = b[:, :2] - a[:, :2], c[:, :2] - a[:, :2]
    v2 = np.array([x, y]) - a[:, :2]
    den = v0[:, 0] * v1[:, 1] - v1[:, 0] * v0[:, 1]
    ok = np.abs(den) > 1e-12
    den = np.where(ok, den, 1.0)
    u = (v2[:, 0] * v1[:, 1] - v1[:, 0] * v2[:, 1]) / den
    w = (v0[:, 0] * v2[:, 1] - v2[:, 0] * v0[:, 1]) / den
    inside = ok & (u >= -1e-9) & (w >= -1e-9) & (u + w <= 1 + 1e-9)
    idx = np.flatnonzero(inside)
    if idx.size == 0:
        return None
    i = idx[np.argmax(a[idx, 2] + u[idx] * (b[idx, 2] - a[idx, 2]) + w[idx] * (c[idx, 2] - a[idx, 2]))]
    z = a[i, 2] + u[i] * (b[i, 2] - a[i, 2]) + w[i] * (c[i, 2] - a[i, 2])
    nrm = np.cross(b[i] - a[i], c[i] - a[i])
    nrm /= np.linalg.norm(nrm)
    if nrm[2] < 0:
        nrm = -nrm
    return float(z), nrm


def zenoh_router_needed():
    if os.environ.get('RMW_IMPLEMENTATION', '') != 'rmw_zenoh_cpp':
        return False
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.3)
        return s.connect_ex(('127.0.0.1', 7447)) != 0


def launch_setup(context, *args, **kwargs):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    def flag(name):
        return arg(name).lower() in ('true', '1', 'yes')

    share = get_package_share_directory(PKG)
    trees = arg('trees')
    if trees not in VARIANTS:
        raise RuntimeError(f"trees:={trees} invalido; use 15 ou 68")
    world_file, model_name = VARIANTS[trees]
    world = arg('world') or os.path.join(share, 'worlds', world_file)

    resource_dirs = [os.path.join(share, 'models')]
    if trees == '68':
        resource_dirs.append(arg('cerrado68_models'))
    model_dir = next((os.path.join(d, model_name) for d in resource_dirs
                      if os.path.isdir(os.path.join(d, model_name))), None)
    if model_dir is None:
        raise RuntimeError(f"modelo '{model_name}' nao encontrado em {resource_dirs}")

    # ---- pose de spawn sobre o terreno ----
    x, y, yaw = float(arg('x')), float(arg('y')), float(arg('yaw'))
    roll = pitch = 0.0
    info = []
    if arg('z') == 'auto':
        tp = terrain_pose(os.path.join(model_dir, 'meshes', 'terrain_collision.stl'), x, y)
        if tp is None:
            raise RuntimeError(f"({x}, {y}) esta fora do terreno (x/y devem estar em [-16, 16])")
        z, nrm = tp
        # normal do terreno no referencial girado pelo yaw -> roll/pitch que alinham o robo
        nx = math.cos(yaw) * nrm[0] + math.sin(yaw) * nrm[1]
        ny = -math.sin(yaw) * nrm[0] + math.cos(yaw) * nrm[1]
        roll = -math.asin(max(-1.0, min(1.0, ny)))
        pitch = math.atan2(nx, nrm[2])
        slope = math.degrees(math.acos(max(-1.0, min(1.0, nrm[2]))))
        info.append(LogInfo(msg=f"[limo_cerrado] terreno em ({x:.2f}, {y:.2f}): z={z:.3f} m, "
                                f"inclinacao {slope:.1f} graus"))
        z += SPAWN_CLEARANCE
    else:
        z = float(arg('z'))

    # ---- Gazebo ----
    env = {'GZ_SIM_RESOURCE_PATH': os.pathsep.join(
        resource_dirs + [p for p in [os.environ.get('GZ_SIM_RESOURCE_PATH', '')] if p])}
    if flag('nvidia'):
        env['__NV_PRIME_RENDER_OFFLOAD'] = '1'
        env['__GLX_VENDOR_LIBRARY_NAME'] = 'nvidia'
    gz_cmd = ['gz', 'sim', '-r', '-v', arg('gz_verbosity'), world]
    if not flag('gui'):
        gz_cmd.append('-s')
    if flag('headless_rendering'):
        gz_cmd.append('--headless-rendering')
    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        # O gz resolve "model://<nome>" primeiro contra o diretorio atual. Rodando de /root,
        # "model://cerrado_32x32" cairia na pasta /root/cerrado_32x32 (sem model.config) e o
        # mundo nao carregaria. No share do pacote nao existe pasta com o nome do modelo.
        cwd=share,
        additional_env=env,
        output='screen',
    )

    # ---- robo ----
    xacro_cmd = ['xacro ', os.path.join(share, 'urdf', 'limo_pro_gz.urdf.xacro')]
    if arg('mode') not in ('track', 'diff'):
        raise RuntimeError(f"mode:={arg('mode')} invalido; use track ou diff")
    for name in ('mode', 'wheel_friction', 'steering_efficiency', 'use_livox', 'use_lidar2d', 'use_camera',
                 'physical_inertia', 'detailed_collision', 'livox_xyz', 'livox_rpy'):
        xacro_cmd += [f' {name}:="', arg(name), '"']
    robot_description = ParameterValue(Command(xacro_cmd), value_type=str)

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', WORLD_NAME, '-topic', 'robot_description', '-name', ROBOT_NAME,
                   '-x', f'{x:.4f}', '-y', f'{y:.4f}', '-z', f'{z:.4f}',
                   '-R', f'{roll:.5f}', '-P', f'{pitch:.5f}', '-Y', f'{yaw:.5f}'],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                     'use_sim_time': True}],
    )

    actions = info + [gazebo, rsp, spawn, bridge]

    if arg('zenoh_router') == 'true' or (arg('zenoh_router') == 'auto' and zenoh_router_needed()):
        actions.insert(0, LogInfo(msg='[limo_cerrado] RMW=rmw_zenoh_cpp sem roteador: subindo rmw_zenohd'))
        actions.insert(1, Node(package='rmw_zenoh_cpp', executable='rmw_zenohd', output='log'))

    if flag('rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', output='log',
            arguments=['-d', os.path.join(share, 'rviz', 'limo_cerrado.rviz')],
            parameters=[{'use_sim_time': True}],
            additional_env={k: v for k, v in env.items() if k.startswith('__')},
        ))
    return actions


def generate_launch_description():
    args = [
        ('mode', 'track', 'locomocao: track (esteiras) ou diff (quatro rodas do modelo oficial)'),
        ('wheel_friction', 'skid', 'so no modo diff: skid (atrito lateral reduzido) ou official (atrito do URDF oficial; nao gira no lugar)'),
        ('steering_efficiency', '0.7', 'so no modo track: eficiencia de giro do TrackedVehicle'),
        ('trees', '15', 'variante do mundo: 15 ou 68 arvores'),
        ('world', '', 'caminho de um .sdf alternativo (vazio = mundo da variante)'),
        ('cerrado68_models', CERRADO68_MODELS_DEFAULT, 'pasta "models" do mapa de 68 arvores'),
        # clareira plana (inclinacao ~3 graus) com 3,3 m livres nos dois mundos
        ('x', '-13.0', 'spawn x [m]'),
        ('y', '11.5', 'spawn y [m]'),
        ('z', 'auto', 'spawn z [m]; auto = altura do terreno em (x, y)'),
        ('yaw', '-0.72', 'spawn yaw [rad] (padrao: olhando para o centro do mapa)'),
        ('gui', 'true', 'abre a GUI do Gazebo'),
        ('rviz', 'false', 'abre o RViz'),
        ('nvidia', 'true', 'renderiza na GPU NVIDIA (PRIME offload)'),
        ('headless_rendering', 'false', 'renderizacao EGL sem display (com gui:=false)'),
        ('gz_verbosity', '2', 'verbosidade do gz sim (0-4)'),
        ('zenoh_router', 'auto', 'auto | true | false: sobe o rmw_zenohd se o RMW for zenoh e nao houver roteador'),
        ('use_livox', 'true', 'Livox MID-360 em /livox/lidar e /livox/imu'),
        ('use_lidar2d', 'true', 'lidar 2D de fabrica (EAI T-mini Pro) em /scan'),
        ('use_camera', 'false', 'camera de profundidade de fabrica (Orbbec Dabai) em /camera/*'),
        ('physical_inertia', 'false', 'true = inercia de caixa homogenea; false = inercia do URDF oficial'),
        ('detailed_collision', 'false', 'true = colisao da base seguindo o mesh; false = caixa do URDF oficial'),
        ('livox_xyz', '0.0 0.0 0.151', 'posicao do MID-360 em relacao ao base_link'),
        ('livox_rpy', '0 0 0', 'orientacao do MID-360 em relacao ao base_link'),
    ]
    return LaunchDescription(
        [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in args]
        + [OpaqueFunction(function=launch_setup)])
