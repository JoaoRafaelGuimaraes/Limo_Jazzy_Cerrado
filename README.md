# limo_ws — AgileX LIMO Pro + Livox MID-360 no mundo Cerrado

Workspace independente (ROS 2 Jazzy + Gazebo Harmonic). Nao usa nem altera o `mrs_ws`.

## Conteudo

| Caminho | O que e |
|---|---|
| `src/limo_ros2/` | Clone intocado do repositorio oficial [agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) (branch `humble`). So o pacote `limo_description` e compilado: URDF/xacro e meshes oficiais. |
| `src/limo_cerrado_sim/` | Pacote deste projeto: xacro do LIMO Pro para o Harmonic, mundos, copia do modelo do cerrado (15 arvores), bridge, launch e RViz. |
| `build.sh` | Compila apenas `limo_description` + `limo_cerrado_sim`. |

`limo_base` e `limo_car` do repo oficial nao sao compilados: dependem do hardware e do Gazebo Classic.

## Compilar e rodar

```bash
cd /root/limo_ws && ./build.sh
source /root/limo_ws/install/setup.bash
ros2 launch limo_cerrado_sim limo_cerrado.launch.py
```

Em outro terminal (com o mesmo `source`), para dirigir:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```



### Argumentos do launch

| Argumento | Padrao | Efeito |
|---|---|---|
| `mode` | `track` | `track` = esteiras; `diff` = quatro rodas do modelo oficial. |
| `wheel_friction` | `skid` | So no modo `diff`: `skid` (atrito lateral reduzido) ou `official` (atrito do URDF oficial). |
| `steering_efficiency` | `0.7` | So no modo `track`: eficiencia de giro do plugin de esteiras. |
| `trees` | `15` | `15` ou `68` arvores. O modelo de 68 (~500 MB) nao e copiado: vem de `/root/cerrado_32x32/models` (`cerrado68_models:=`). |
| `x`, `y`, `yaw` | `-13.0`, `11.5`, `-0.72` | Spawn. O padrao e uma clareira com ~3 graus de inclinacao e 3,3 m livres nos dois mundos. |
| `z` | `auto` | Altura lida do mesh do terreno; o robo nasce alinhado a inclinacao local. |
| `gui`, `rviz` | `true`, `false` | GUI do Gazebo e RViz. |
| `nvidia` | `true` | Renderiza na GPU NVIDIA (PRIME offload). |
| `use_livox` | `true` | Livox MID-360. |
| `use_lidar2d` | `true` | Lidar 2D de fabrica (EAI T-mini Pro). |
| `use_camera` | `false` | Camera de profundidade de fabrica (Orbbec Dabai). |
| `livox_xyz`, `livox_rpy` | `0 0 0.151`, `0 0 0` | Montagem do MID-360 em relacao ao `base_link` (padrao: topo da carenagem). |
| `physical_inertia` | `false` | `true` = inercia de caixa homogenea; `false` = inercia do URDF oficial. |
| `detailed_collision` | `false` | `true` = colisao da base seguindo o mesh; `false` = caixa do URDF oficial. |
| `zenoh_router` | `auto` | Sobe o `rmw_zenohd` se o RMW for `rmw_zenoh_cpp` e nao houver roteador. |
| `headless_rendering` | `false` | Com `gui:=false`, renderiza os sensores por EGL sem display. |

## Topicos ROS

| Topico | Tipo | Origem |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | comando (limite 1 m/s, como o LIMO) |
| `/odom`, `/tf` (`odom -> base_footprint`) | `nav_msgs/Odometry` | odometria do plugin de tracao (esteiras ou rodas) |
| `/ground_truth/odom` | `nav_msgs/Odometry` | pose verdadeira 3D no mundo |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | MID-360, frame `livox_frame`, 10 Hz |
| `/livox/imu` | `sensor_msgs/Imu` | IMU do MID-360, 200 Hz |
| `/scan` | `sensor_msgs/LaserScan` | T-mini Pro, frame `laser_link` |
| `/imu` | `sensor_msgs/Imu` | IMU de fabrica (HI226), 100 Hz |
| `/camera/*` | imagens, `CameraInfo`, nuvem | Dabai, so com `use_camera:=true` |
| `/joint_states`, `/robot_description` | | rodas (so no modo `diff`) e modelo para o RViz |

## Pose verdadeira e comparacao com SLAM/LIO

### De onde vem o `/ground_truth/odom`

Nao vem de sensor nem de estimador. O motor de fisica do Gazebo conhece a pose exata do robo a cada passo. O plugin `OdometryPublisher` do gz-sim, preso ao robo no fim de `urdf/limo_pro_gz.urdf.xacro`, le essa pose e a publica a 50 Hz, em 3D e sem ruido, no topico Gazebo `/model/limo/ground_truth/odom`. O `ros_gz_bridge` (`config/bridge.yaml`) converte para `nav_msgs/Odometry` em `/ground_truth/odom`.

- `header.frame_id = world`: o referencial do mundo do Gazebo, o mesmo do SDF (x/y em [-16, 16], z para cima, origem no centro do terreno).
- `child_frame_id = base_footprint`: o ponto no chao sob o centro do robo. **Nao e a pose do lidar.**
- Carimbo em tempo de simulacao. So existe na simulacao; o robo real nao tem esse topico.
- O referencial `world` nao entra no TF. O TF so tem `odom -> base_footprint`, da odometria das esteiras, que e plana (sem z, roll e pitch) e comeca em zero no ponto de spawn.

### Onde o robo nasce

| | Padrao | Como mudar |
|---|---|---|
| x, y | -13.0, 11.5 m | `x:=` `y:=` |
| yaw | -0.72 rad (-41,3 graus), olhando para o centro do mapa | `yaw:=` |
| z | altura do terreno em (x, y) + 0,06 m; no padrao, terreno a 1,47 m | `z:=` (padrao `auto`) |
| roll, pitch | alinhados a inclinacao local do terreno | automatico com `z:=auto` |

Pose medida depois de o robo assentar no spawn padrao: x = -13,000, y = 11,500, z = 1,474 m, roll = 0,1, pitch = -1,1, yaw = -41,5 graus. Como o spawn e configuravel e o robo assenta alguns milimetros, **nao use esses numeros fixos na avaliacao: use a primeira mensagem de `/ground_truth/odom` como pose inicial.**


## Fidelidade ao modelo oficial

O clone em `src/limo_ros2/` nao e alterado. `urdf/limo_pro_gz.urdf.xacro` espelha o `limo_four_diff.xacro` oficial e usa, sem mudanca, as macros, os meshes, a geometria, as massas, a inercia e a caixa de colisao da base. Por padrao so muda o que e obrigatorio:

- **Plugins.** Os oficiais sao do Gazebo Classic (`libgazebo_ros_*`) e nao existem no Harmonic. Entram os nativos do `gz-sim`.
- **Pose da camera.** O `limo_ros2` traz z = 0,3 (camera a 45 cm do solo, acima do robo). O outro repo oficial, `ugv_gazebo_sim`, traz 0,03, que bate com o mesh. Usado 0,03.
- **Atrito das rodas (modo `diff`).** Os valores oficiais (mu = 100, `kp`, `kd`, `minDepth`) sao ajustes para o motor de fisica do Classic. No Harmonic/DART, com eles o robo nao gira. `wheel_friction:=official` os restaura.

Acrescimos: o Livox MID-360 e o modo esteira. Desvios opcionais, desligados por padrao: `physical_inertia` e `detailed_collision`.

## Modos de locomocao

- **`mode:=track` (padrao).** Duas esteiras no lugar das quatro rodas, como o modo esteira do LIMO real. A AgileX nao publica URDF desse modo: as esteiras foram montadas sobre a geometria oficial (mesmos eixos, mesma bitola de 0,175 m, massa igual a das rodas substituidas, raio 0,05 m). As rodas oficiais continuam como visual. Usa os sistemas `TrackedVehicle` e `TrackController` do gz-sim.
- **`mode:=diff`.** Quatro rodas do modelo oficial com o plugin `DiffDrive`.

Medido neste terreno (pose verdadeira, comando de 1,0 rad/s parado e 0,4 m/s + 0,8 rad/s em arco):

| Configuracao | Giro no lugar | Arco | Deriva durante o giro | Subida de 18,5 graus (pico 22,5) |
|---|---|---|---|---|
| `track` | 1,03 a 1,13 rad/s | 0,82 rad/s | 2 a 3 cm | 0,38 m/s, segura parado |
| `diff` + `skid` | 0,82 a 0,89 rad/s | 0,44 rad/s | 12 a 18 cm | 0,38 m/s, segura parado |
| `diff` + `official` | 0,00 rad/s | 0,20 rad/s | 21 a 34 cm | 0,38 m/s, segura parado |

Leituras:

- A subida e igual nas tres. O limite e o atrito do solo do mundo (mu = 0,85, cerca de 40 graus), e o terreno chega a 30 graus. Nesta simulacao a esteira nao sobe mais que a roda; o ganho dela e na direcao.
- A esteira gira sem sair do lugar. As rodas, ao contra-rodar, perdem aderencia e o robo escorrega ladeira abaixo.
- A taxa de giro da esteira varia com a inclinacao local (medido ate 1,6 rad/s para 1,0 comandado em outro ponto).
- A odometria de giro e ruim nos dois modos, como no robo real: a esteira reporta 1,43 rad/s girando a 1,1; as rodas reportam o comandado girando a 0,85. Avanco e re batem com a odometria (erro abaixo de 1%). Para pose use `/ground_truth/odom` ou LIO com o Livox.

## Livox MID-360

Aproximado com `gpu_lidar`: 360 x 59 graus (-7 a +52), 0,1 a 40 m, 10 Hz, 500 x 40 = 20 mil pontos por quadro, mais a IMU interna a 200 Hz. Limites: a varredura e em grade uniforme, nao o padrao nao repetitivo do sensor real; a mensagem e `PointCloud2` com campos `x y z intensity ring`, sem tempo por ponto e sem `livox_ros_driver2/CustomMsg`; a IMU sai em m/s^2 (o driver real publica em g). Isso importa para Point-LIO/FAST-LIO. Num campo aberto cerca de 17% dos raios retornam; o resto aponta para o ceu.

## Mundo

`worlds/cerrado_15.sdf` e `cerrado_68.sdf`: terreno em malha de 32 x 32 m (x/y em [-16, 16]), relevo de 0 a 2,7 m, inclinacao mediana de 7 graus e maxima de 30. Nao ha chao fora do quadrado: o robo cai se sair da borda. Troncos, galhos, raizes, arbustos e galhos caidos tem colisao; folhas e capim sao so visuais.
