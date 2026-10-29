# Atividade de ROS 2 — Sistema de Teste dos Motores do AUV

Pacote ROS 2 (Python) para testar individualmente e em conjunto os 7 motores do AUV, com interface gráfica.

## Estrutura

```
ros2-teste-motores/
├── motor_test_interfaces/        # definição das actions (ament_cmake)
│   └── action/
│       ├── TestMotor.action      # teste individual
│       └── TestAllMotors.action  # teste completo
└── motor_test/                   # nós (ament_python)
    ├── motor_test/
    │   ├── motor_control_node.py # controle dos motores + action servers
    │   └── gui_node.py           # interface gráfica (Tkinter)
    └── launch/
        └── motor_test.launch.py  # inicia os dois nós
```

## Como funciona

**Nó de controle (`motor_control_node`)**
- Publica o PWM dos 7 motores em `/motors` (`std_msgs/Int32MultiArray`, 7 elementos).
- PWM neutro = 1500. Relação linear PWM → força: 1000 → −10 N, 1500 → 0 N, 2000 → +10 N.
- Action `test_motor`: recebe o motor (0–6), aciona por 2 s com os outros em neutro; o feedback indica o motor ativo e o tempo decorrido.
- Action `test_all_motors`: aciona cada motor sozinho por 2 s e depois todos juntos por 2 s (8 etapas); o feedback indica os motores ativos, os já testados e a etapa atual.
- Só um teste roda por vez: se um teste estiver em andamento, qualquer outro pedido (individual ou completo) é **recusado**.
- Ao terminar, cancelar ou falhar, todos os motores voltam para 1500. O mesmo acontece ao desligar o nó.

**Interface gráfica (`gui_node`)**
- Um botão por motor, com o estado de cada teste (testando / concluído / falha).
- Botão **Testar todos**, com a etapa atual e a lista de motores já testados.
- Botão **Cancelar** para interromper o teste em andamento.
- Painel com o PWM atual de cada motor, lido de `/motors`.

**Parâmetros do nó de controle**

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `test_pwm` | 1600 | PWM aplicado durante o teste (+2 N) |
| `test_duration` | 2.0 | Duração de cada teste (s) |
| `num_motors` | 7 | Número de motores |
| `publish_rate` | 10.0 | Frequência de publicação em `/motors` (Hz) |

## Como rodar

Testado com ROS 2 Humble.

```bash
# dentro do src/ de um workspace
cd ~/ros2_ws/src
cp -r <este-repo>/ros2-teste-motores .
cd ~/ros2_ws
colcon build
source install/setup.bash

ros2 launch motor_test motor_test.launch.py
# opcional: ros2 launch motor_test motor_test.launch.py test_pwm:=1700
```

Se faltar o Tkinter: `sudo apt install python3-tk`.

## Testando pelo terminal (sem a interface)

```bash
ros2 run motor_test motor_control_node

# em outro terminal
ros2 topic echo /motors
ros2 action send_goal --feedback /test_motor motor_test_interfaces/action/TestMotor "{motor_id: 3}"
ros2 action send_goal --feedback /test_all_motors motor_test_interfaces/action/TestAllMotors "{}"
```
