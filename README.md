# control_node — Controlador PID com controle de torque (MG8008E-i9)

Nó ROS 2 em C++ que fecha a malha de controle **IDB+PID** com **controle de
torque** sobre o motor **K-Tech MG8008E-i9** via **CAN**, na Raspberry Pi 4.
Integra-se à arquitetura ROS 2 do projeto de órtese de joelho.

## Arquitetura

```
/control/reference ──▶ [control_node] ──CAN(0xA1)──▶ [MG8008E-i9] ──▶ junta do joelho
   [θref,θ̇ref,θ̈ref]        │  PID(IDB)        ◀──CAN(resp)──── encoder interno (θ)
                            │
                            └──▶ /control/state [θ, τ, erro, τ_est, temp]

/control/set_torque (SetBool) ──▶ habilita/desabilita o torque (segurança)
```

O nó segue o mesmo padrão dos nós do Márcio (`actuator_node`, `imu_node`):
parâmetros via `.yaml`, um tópico de estado para telemetria, e um serviço
para operação pontual (aqui, ligar/desligar o torque).

## Por que controle de torque (e não posição)

O MG8008E-i9 é um motor *quasi-direct-drive* cujo driver aceita comando de
**corrente** (torque) diretamente via CAN (comando `0xA1`). Isso permite
implementar o PID que gera torque — a arquitetura validada no TCC — em vez de
depender de uma malha de posição interna como a do Dynamixel. O ângulo de
realimentação vem do **encoder interno do próprio motor**, que chega na
resposta de cada comando de torque (uma única transação CAN por ciclo).

## Estrutura dos arquivos

| Arquivo | Função |
|---|---|
| `include/control_node/pid_controller.hpp` | Classe PID (IDB+PID), header-only |
| `include/control_node/mg_motor_can.hpp` | Interface do driver CAN |
| `src/mg_motor_can.cpp` | Driver CAN (SocketCAN), comandos 0xA1/0x9C/0x88/0x80 |
| `src/control_node.cpp` | Nó ROS 2 (loop de controle, tópicos, serviço) |
| `config/control.yaml` | Parâmetros (ganhos, limites, CAN) |
| `launch/control.launch.py` | Launch |

## Pré-requisitos na Raspberry Pi

### 1. Configurar a interface CAN

A Raspberry Pi precisa de um transceptor CAN (ex.: MCP2515 no SPI, ou um
HAT CAN). Depois de instalado:

```bash
sudo ip link set can0 type can bitrate 1000000   # 1 Mbps (padrão RMD)
sudo ip link set up can0
```

Verifique com `candump can0` (do pacote `can-utils`).

### 2. Valores do motor (já preenchidos do datasheet)

O `config/control.yaml` já vem com os valores do datasheet MG8008E-i9 V3:

```yaml
tau_max: 20.0     # Max Torque [N.m]
kt: 2.0           # Torque Constant [N.m/A] no eixo de saída (rotor 0.22 x 9)
i_max: 12.0       # Max Current [A]
gear_ratio: 9.0   # redução 9:1
temp_limit: 70    # corte de segurança [C]
```

O ângulo do joelho é lido via **Read Multi Loop Angle** (comando `0x92`), que
retorna o ângulo do eixo de saída já acumulado — não depende de decodificar
os bits crus do encoder.

### CALIBRAÇÃO PENDENTE (escala comando→corrente)

Um único parâmetro ainda precisa de calibração elétrica: `raw_max` no
`mg_motor_can.hpp` (default 2000), que mapeia `i_max` para o valor bruto do
comando de torque `0xA1`. Para calibrar:

1. Motor preso contra uma carga (braço com peso conhecido, ou travado).
2. Fonte em 24 V, limite de corrente ≥ 12 A.
3. Mande torque conhecido e meça a corrente real (fonte ou IA/IB/IC).
4. Ajuste `raw_max` até que o torque comandado bata com o medido
   (torque medido = corrente × 2,0 N·m/A).

A saturação em `tau_max=20 N·m` protege o motor mesmo antes da calibração.

## Compilação (no workspace ROS 2 / Docker)

Coloque a pasta `control_node` em `src/` do workspace e:

```bash
colcon build --symlink-install --packages-up-to control_node
source install/setup.bash
```

## Execução

```bash
ros2 launch control_node control.launch.py
```

O nó inicia com o **torque desabilitado** por segurança. Para habilitar:

```bash
ros2 service call /control/set_torque std_srvs/srv/SetBool "{data: true}"
```

Para desabilitar (zera o torque e desliga o motor):

```bash
ros2 service call /control/set_torque std_srvs/srv/SetBool "{data: false}"
```

### Enviar uma referência de marcha

A referência é `[θref, θ̇ref, θ̈ref]` em rad, rad/s, rad/s². Exemplo de um
ângulo fixo de −20° (0,349 rad negativo):

```bash
ros2 topic pub /control/reference std_msgs/msg/Float64MultiArray \
  "{data: [-0.349, 0.0, 0.0]}"
```

Na prática, um nó separado (ou o `sync_node`) publica a trajetória de marcha
ponto a ponto a cada ciclo, derivada do `.mot` do OpenSim já filtrado a 6 Hz.

### Ler a telemetria

```bash
ros2 topic echo /control/state
# [theta_rad, tau_Nm, erro_rad, torque_estimado_Nm, temperatura_C]
```

## Como integrar com a arquitetura do Márcio

Este nó usa interfaces genéricas (`Float64MultiArray`, `SetBool`) para não
depender das interfaces customizadas do projeto. Para integração plena:

1. **Reusar `/actuator/state`**: troque o publisher por
   `interfaces/msg/ActuatorState` (mas note que ela usa `int16[] positions`,
   adequada a posição, não torque — talvez convenha criar uma
   `ControlState.msg` com `float64`).

2. **Receber a marcha do `sync_node`**: subscreva `/sync/data` e extraia o
   ângulo de referência de onde sua trajetória estiver.

3. **Serviço de torque**: o Márcio já tem `/actuator/set_torque`
   (`interfaces/srv/SetTorque`); você pode alinhar a assinatura do serviço com
   a dele em vez de `std_srvs/SetBool`.

## Segurança

- O nó inicia com torque **desabilitado**; nada se move até o serviço ser
  chamado.
- A saturação em `tau_max=20 N·m` está ativa em todos os caminhos.
- No encerramento (Ctrl-C), o destrutor zera o torque e desliga o motor.
- Recomenda-se um botão de emergência físico em série com a alimentação do
  motor, independente do software.

## Validação já feita

A classe `PIDController` foi compilada e testada isoladamente:
- Degrau de 45°: atinge a referência com erro de regime nulo.
- Saturação em 20 N·m: ativa e correta.
- Marcha 1 Hz com IDB+PID e `tau_max=20`: RMS ≈ 0,19° (sub-grau).

O comportamento espelha exatamente os modelos MATLAB/Simulink do TCC.

## Deploy na Raspberry Pi (GitHub + VNC + Docker)

### 1. Subir o código para o GitHub (no seu PC)

```bash
cd control_node
git init
git add .
git commit -m "control_node: PID torque via CAN para MG8008E-i9"
git remote add origin https://github.com/SEU_USUARIO/ortese-control-node.git
git branch -M main
git push -u origin main
```

### 2. Acessar a Raspberry via RealVNC

Abra o RealVNC Viewer no PC do laboratório, conecte no IP da Pi (já
cadastrado) e use a senha do lab. Você verá a tela da Raspberry.

### 3. Clonar na Pi (desativando proxy do lab)

```bash
# o proxy do lab costuma bloquear o GitHub: desative antes
unset http_proxy https_proxy
git config --global --unset http.proxy 2>/dev/null
git config --global --unset https.proxy 2>/dev/null

cd ~/sensorhub/src    # ou onde estiver o workspace ROS 2
git clone https://github.com/SEU_USUARIO/ortese-control-node.git control_node
```

### 4. Configurar a interface CAN

```bash
sudo ip link set can0 type can bitrate 1000000   # 1 Mbps (default do motor)
sudo ip link set up can0
candump can0   # teste: deve mostrar trafego se o motor estiver ligado
```

### 5. Compilar (dentro do Docker, como o projeto do Márcio)

```bash
cd ~/sensorhub/dev
sudo docker compose run --rm sensorhub
# dentro do container:
cd /app/ros2_ws
colcon build --symlink-install --packages-up-to control_node
source install/setup.bash
```

### 6. Executar

```bash
ros2 launch control_node control.launch.py
# em outro terminal, habilitar o torque (motor começa desabilitado):
ros2 service call /control/set_torque std_srvs/srv/SetBool "{data: true}"
```
