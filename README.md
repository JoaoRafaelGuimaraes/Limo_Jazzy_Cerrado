# limo_ws — AgileX LIMO Pro + Livox MID-360 no mundo Cerrado

Workspace independente (ROS 2 Jazzy + Gazebo Harmonic). Nao usa nem altera o `mrs_ws`.

## Conteudo

| Caminho | O que e |
|---|---|
| `src/limo_ros2/` | Clone intocado do repositorio oficial [agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) (branch `humble`). So o pacote `limo_description` e compilado: URDF/xacro e meshes oficiais. |
| `src/limo_cerrado_sim/` | Pacote deste projeto: xacro do LIMO Pro para o Harmonic, mundos, copia do modelo do cerrado (15 arvores), bridge, launch e RViz. |
| `build.sh` | Compila apenas `limo_description` + `limo_cerrado_sim`. |

`limo_base` e `limo_car` do repo oficial nao sao compilados: dependem do hardware e do Gazebo Classic.

## Instalacao rapida

Requisitos: Ubuntu 24.04 e [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

**1. Dependencias** (o `ros-jazzy-ros-gz` ja traz o Gazebo Harmonic):

```bash
sudo apt update
sudo apt install git git-lfs curl python3-colcon-common-extensions \
  ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-joint-state-publisher ros-jazzy-rviz2 ros-jazzy-depth-image-proc \
  ros-jazzy-teleop-twist-keyboard ros-jazzy-grid-map-rviz-plugin
```

**2. Clonar e compilar:**

```bash
git clone --recursive https://github.com/JoaoRafaelGuimaraes/Limo_Jazzy_Cerrado.git /root/limo_ws
cd /root/limo_ws
git lfs install && git lfs pull   # meshes do terreno e do robo
./build.sh                        # compila e baixa o padrao de varredura do MID-360
```

**3. Rodar:**

```bash
source /root/limo_ws/install/setup.bash
ros2 launch limo_cerrado_sim limo_cerrado.launch.py
```

Em outro terminal (com o mesmo `source`), para dirigir:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Para ver os sensores no RViz: `rviz:=true`.



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
| `lidar_sees_grass` | `false` | `false` = o capim fica invisivel para o Livox e para o lidar 2D. A GUI e a camera continuam vendo. |
| `use_livox` | `true` | Livox MID-360. Ver a secao propria para `livox_model`, `livox_format` e `livox_motion_distortion`. |
| `use_lidar2d` | `true` | Lidar 2D de fabrica (EAI T-mini Pro). |
| `use_camera` | `true` | Camera RGB-D. Custa cerca de 6% do tempo real; `false` devolve 1,0. |
| `camera_model` | `d435` | `d435` = Intel RealSense D435/D435i; `dabai` = Orbbec Dabai, a do manual do LIMO Pro. |
| `camera_rate` | `30` | Taxa da camera em Hz. |
| `camera_pointcloud` | `true` | Gera a nuvem colorida a partir da profundidade. |
| `camera_xyz`, `camera_rpy` | `0.084 0 0.03`, `0 0 0` | Montagem da camera em relacao ao `base_link` (padrao: a do modelo oficial). |
| `livox_xyz`, `livox_rpy` | `0 0 0.151`, `0 0 0` | Montagem do MID-360 em relacao ao `base_link` (padrao: topo da carenagem). |
| `physical_inertia` | `false` | `true` = inercia de caixa homogenea; `false` = inercia do URDF oficial. |
| `detailed_collision` | `false` | `true` = colisao da base seguindo o mesh; `false` = caixa do URDF oficial. |
| `zenoh_router` | `auto` | Sobe o `rmw_zenohd` se o RMW for `rmw_zenoh_cpp` e nao houver roteador. |
| `headless_rendering` | `false` | Com `gui:=false`, renderiza os sensores por EGL sem display. |

## Topicos ROS

| Topico | Tipo | Origem |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | comando (limite 1 m/s, como o LIMO) |
| `/tf` (`odom -> base_footprint`) | `tf2_msgs/TFMessage` | pose 3D do robo (x, y, z, roll, pitch, yaw), ver "Odometria e TF" |
| `/odom` | `nav_msgs/Odometry` | odometria de rodas/esteiras, plana. So para comparacao: nao vai para o TF |
| `/ground_truth/odom` | `nav_msgs/Odometry` | pose verdadeira 3D no mundo |
| `/livox/lidar` | `PointCloud2` (ou `CustomMsg`) | MID-360 com padrao de varredura real e tempo por ponto, frame `livox_frame`, 10 Hz |
| `/livox/imu` | `sensor_msgs/Imu` | IMU do MID-360, 200 Hz |
| `/scan` | `sensor_msgs/LaserScan` | T-mini Pro, frame `laser_link` |
| `/imu` | `sensor_msgs/Imu` | IMU de fabrica (HI226), 100 Hz |
| `/camera/camera/color/image_raw`, `.../color/camera_info` | `Image` rgb8, `CameraInfo` | RealSense, cor 640x480, 30 Hz |
| `/camera/camera/aligned_depth_to_color/image_raw`, `/camera/camera/depth/image_rect_raw` | `Image` 32FC1 | profundidade em metros, ja alinhada a cor |
| `/camera/camera/depth/color/points` | `PointCloud2` xyz + rgb | nuvem colorida, frame optico |
| `/joint_states`, `/robot_description` | | rodas (so no modo `diff`) e modelo para o RViz |

## Odometria e TF

**Por que importa.** O lidar mede cada ponto em relacao a si mesmo. Para montar um mapa, o TF `odom -> base_footprint` diz onde o robo esta e quanto ele esta inclinado. Se o TF errar a inclinacao, o mapa erra a altura dos pontos: com o robo inclinado 10 graus, um ponto a 10 m sai 1,76 m fora do lugar (`d * tan(10 graus)`).

**O problema da odometria de rodas.** Ela conta o giro das rodas, entao sabe x, y e yaw, mas nao sabe se o robo esta numa rampa: sempre diz z = 0, roll = 0, pitch = 0. Neste terreno inclinado, isso cria picos falsos em qualquer mapa de elevacao.

**O que a simulacao faz.** O TF `odom -> base_footprint` vem do plugin `OdometryPublisher` do Gazebo, com a pose 3D exata do robo. A odometria de rodas continua em `/odom`, so para comparacao.

- A origem do `odom` e o centro do mundo do Gazebo, nao o ponto de spawn.
- Isso e ground truth do simulador: serve para validar algoritmos de mapeamento. No robo real, use SLAM 3D com IMU (LIO, por exemplo FAST-LIO2) para obter z, roll e pitch.

## Pose verdadeira e comparacao com SLAM/LIO

### De onde vem o `/ground_truth/odom`

Nao vem de sensor nem de estimador. O motor de fisica do Gazebo conhece a pose exata do robo a cada passo. O plugin `OdometryPublisher` do gz-sim, preso ao robo no fim de `urdf/limo_pro_gz.urdf.xacro`, le essa pose e a publica a 50 Hz, em 3D e sem ruido, no topico Gazebo `/model/limo/ground_truth/odom`. O `ros_gz_bridge` (`config/bridge.yaml`) converte para `nav_msgs/Odometry` em `/ground_truth/odom`.

- `header.frame_id = world`: o referencial do mundo do Gazebo, o mesmo do SDF (x/y em [-16, 16], z para cima, origem no centro do terreno).
- `child_frame_id = base_footprint`: o ponto no chao sob o centro do robo. **Nao e a pose do lidar.**
- Carimbo em tempo de simulacao. So existe na simulacao; o robo real nao tem esse topico.
- O referencial `world` nao entra no TF. Na simulacao, o `odom` do TF coincide com ele (ver "Odometria e TF").

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

O gz-sim so tem lidar em grade uniforme, e o plugin de simulacao do MID-360 do CTU-MRS (derivado do da Livox) e para Gazebo Classic e ROS Noetic: nao carrega no Harmonic. O que se aproveita dele e o **padrao de varredura real** do sensor, um CSV com 800 mil direcoes (4 s a 200 mil pontos por segundo). O modelo padrao, `livox_model:=mid360`, funciona assim:

1. O `gpu_lidar` renderiza uma grade densa de 1201 x 199 raios (0,3 grau por celula) no campo de visao exato do sensor, -7,21 a +52,16 graus. Ela sai em `/livox/dense_points`, de uso interno.
2. O no `livox_mid360_emulator` (C++, `src/`) amostra essa grade nas proximas 20 mil direcoes do padrao a cada quadro de 0,1 s, carimba cada ponto com o seu tempo de captura (5 us por ponto) e publica `/livox/lidar` no formato do `livox_ros_driver2`.
3. **Distorcao de movimento.** O Gazebo renderiza o quadro num instante so, mas os pontos saem com tempos espalhados por 0,1 s. Um LIO corrige a nuvem por esses tempos; se a geometria fosse instantanea, a correcao a estragaria. Por isso cada ponto e reexpresso no referencial do sensor no seu instante de captura, usando a pose verdadeira do robo. O quadro e publicado ao fim da janela, 0,1 s depois do carimbo, como no sensor real.

| Argumento | Padrao | Efeito |
|---|---|---|
| `livox_model` | `mid360` | `mid360` = padrao real + tempo por ponto; `grid` = grade uniforme 500 x 40, sem tempo por ponto. |
| `livox_format` | `pointcloud2` | `pointcloud2` = campos `x y z intensity tag line timestamp`, 26 bytes por ponto, igual ao `xfer_format 0` do driver. `custom` = `livox_ros_driver2/CustomMsg`, igual ao `xfer_format 1`. |
| `livox_motion_distortion` | `true` | Desligue para obter a nuvem instantanea (os tempos por ponto continuam nominais). |

Medido numa area com objetos, contra a geometria real do cenario (pontos a mais de 3 m):

| Situacao | Erro mediano | p90 |
|---|---|---|
| Parado | 1,9 cm | 4,2 cm |
| Girando a 1 rad/s, sem corrigir | 4,3 cm | 11,7 cm |
| Girando a 1 rad/s, cada ponto na pose do seu proprio carimbo | 1,9 cm | 4,2 cm |

A ultima linha igual a primeira mostra que a distorcao e exatamente a que os carimbos descrevem. Entre quadros consecutivos so 8% das direcoes se repetem (numa grade seriam 100%), e 10 quadros cobrem 6,4 vezes mais direcoes que 1.

O que continua diferente do sensor real: a direcao de cada ponto e arredondada para a celula de 0,3 grau mais proxima; `intensity`/`reflectivity` e constante (100); pontos sem retorno sao omitidos; a IMU (`/livox/imu`) sai em m/s^2 (o driver real publica em g) e fica no mesmo ponto do lidar, entao a extrinseca IMU-lidar de um LIO deve ser zero na simulacao.

O CSV do padrao (25 MB) nao e versionado: o fork de onde ele vem nao declara licenca. O `build.sh` o baixa com `scripts/fetch_livox_pattern.sh` na primeira compilacao. Sem ele o launch avisa e usa `livox_model:=grid`. Para `livox_format:=custom` o pacote precisa ser compilado com o `livox_ros_driver2` no ambiente.

Custo em tempo real, sem GUI: 1,00 so com os lidars; 0,94 so com a camera; 0,87 com tudo ligado, que e o padrao.

## Capim e lidar

O capim do mundo e so visual: 2775 touceiras de fitas finas, sem colisao. O robo atravessa, mas um lidar renderizado o enxerga como parede. Numa moita, 52% dos pontos do Livox e 66% dos pontos do lidar 2D eram capim, o que encheria de obstaculos falsos qualquer mapa de custo.

Por isso o capim esta num visual proprio (`grass`) com `visibility_flags = 65536`, e os dois lidars tem `visibility_mask` sem esse bit. `lidar_sees_grass:=true` devolve a mascara completa.

Medido no mesmo ponto, classificando cada ponto pelo objeto real mais proximo:

| | Capim visivel | Capim invisivel |
|---|---|---|
| Livox: pontos em capim, a mais de 8 cm do solo | 4737 | 150 |
| Lidar 2D: pontos em capim, a mais de 8 cm do solo | 316 | 26 |
| Camera de profundidade: pontos em capim | 8380 | 8380 |

Os poucos pontos que sobram sao folhas de arbusto coladas em laminas de capim, rotuladas pelo vizinho mais proximo. Com o capim fora, os raios passam e atingem o que esta atras: os pontos em solo sobem de 10% para 29% e em arbustos de 29% para 48%.

Cuidado: no campo real o lidar ve o capim. Ajuste o algoritmo com `false` e teste a robustez com `true`. Folhas de arvores e arbustos continuam visiveis aos lidars nos dois casos.

Para o mundo de 68 arvores o pacote traz uma casca, `models/cerrado_32x32_limo`, que usa as malhas de `/root/cerrado_32x32` sem altera-lo. Se o original for regenerado, recrie a casca.

### Raios sem retorno: `-inf` atras, `+inf` na frente

Na nuvem do Livox, um raio sem retorno sai como `+inf` na metade dianteira do sensor e como `-inf` na metade traseira. Numa clareira aberta, 96% dos raios traseiros e 0% dos dianteiros sao `-inf`. Nao ha nada bloqueando o sensor: a metade traseira devolve pontos validos quando ha objetos. E uma particularidade do `gpu_lidar` com 360 graus. Quem descarta valores nao finitos, como o RViz, nao e afetado. Quem distingue os dois sinais, por exemplo para limpar espaco livre num mapa de custo, deve tratar os dois como "sem retorno".

## Mundo

`worlds/cerrado_15.sdf` e `cerrado_68.sdf`: terreno em malha de 32 x 32 m (x/y em [-16, 16]), relevo de 0 a 2,7 m, inclinacao mediana de 7 graus e maxima de 30. Nao ha chao fora do quadrado: o robo cai se sair da borda. Troncos, galhos, raizes, arbustos e galhos caidos tem colisao; folhas e capim sao so visuais (ver "Capim e lidar").
