# control_node — Controle PID com Torque para Órtese de Joelho (MG8008E-i9)

Pacote ROS 2 (C++) que implementa o controlador **IDB+PID** com **controle de
torque** de uma órtese ativa de joelho de 1 grau de liberdade, acionada pelo
motor **K-Tech MG8008E-i9** via **RS485 serial**, rodando na **Raspberry Pi 4**.

Este é o componente de controle do TCC de Engenharia de Computação (UEFS) sobre
modelagem e controle de órtese de joelho. Ele transforma a teoria validada em
MATLAB/Simulink em código embarcado que roda em tempo real no hardware.

---

## Índice

1. [Visão geral](#1-visão-geral)
2. [Como o controle funciona](#2-como-o-controle-funciona)
3. [Arquitetura do software](#3-arquitetura-do-software)
4. [Os parâmetros do motor e como afetam o código](#4-os-parâmetros-do-motor-e-como-afetam-o-código)
5. [Os ganhos do PID](#5-os-ganhos-do-pid)
6. [Como subir e usar na Raspberry Pi](#6-como-subir-e-usar-na-raspberry-pi)
7. [Testando sem o motor](#7-testando-sem-o-motor)
8. [Calibração elétrica pendente](#8-calibração-elétrica-pendente)
9. [Segurança](#9-segurança)
10. [Solução de problemas](#10-solução-de-problemas)

---

## 1. Visão geral

O sistema lê uma trajetória de marcha humana (arquivo `.mot` do OpenSim), e faz
o motor da órtese seguir essa trajetória aplicando o torque calculado por um
controlador PID com feedforward de dinâmica inversa.

```
 arquivo .mot          trajectory_node          control_node            motor
 (marcha do      -->   lê, filtra,        -->   roda o PID,       -->   MG8008E-i9
  OpenSim)             deriva, publica          gera torque             (RS485)
                                                                          |
                       /control/state  <----------------------------------+
                       (telemetria: ângulo, torque, erro, temperatura)
```

Dois nós ROS 2 trabalham juntos:
- **`trajectory_node`** — lê o `.mot`, processa e publica a referência.
- **`control_node`** — recebe a referência, roda o PID, comanda o motor.

---

## 2. Como o controle funciona

### 2.1 A planta (o sistema físico)

A órtese é modelada como um pêndulo de 1 grau de liberdade. A equação de
movimento (Lagrange-Euler) é:

```
tau = D11 * theta_ddot + mgd * cos(theta) + b * theta_dot
```

onde `theta` é o ângulo do joelho, `tau` o torque do motor, `D11` a inércia,
`mgd` o efeito da gravidade e `b` o atrito. O único termo não-linear é o
`cos(theta)` (gravidade).

### 2.2 A lei de controle: IDB+PID

O controlador combina dois termos (arquitetura validada no TCC):

```
tau = tau_FF + tau_PID
```

**Feedforward (IDB — Inverse Dynamics-Based):** antecipa o torque que a
trajetória exige, calculando a dinâmica inversa da própria planta:

```
tau_FF = D11 * theta_ddot_ref + mgd * cos(theta_ref) + b * theta_dot_ref
```

**Feedback (PID):** corrige o que o feedforward não capturou (perturbações,
imprecisão do modelo), usando o erro entre a referência e o ângulo medido:

```
e = theta_ref - theta_medido
tau_PID = Kp*e + Ki*integral(e) + Kd*de/dt
```

O feedforward faz o "trabalho pesado" (cancela a gravidade), deixando o PID
livre para corrigir só o resíduo. É por isso que o erro de rastreio fica em
torno de 0,06° na simulação, contra ~1,8° de um PID puro.

### 2.3 Onde isso está no código

Toda essa matemática está em `include/control_node/pid_controller.hpp`, na
função `update()`. O código espelha exatamente as equações acima, com dois
cuidados de implementação:
- **derivativo filtrado** (evita o "derivative kick" quando a referência salta);
- **anti-windup** (impede o integrador de crescer demais quando o torque satura).

---

## 3. Arquitetura do software

### 3.1 Estrutura de arquivos

```
control_node/
├── CMakeLists.txt                       # build (compila os 2 executáveis)
├── package.xml                          # metadados ROS 2
├── .gitignore                           # ignora build/, install/, log/
├── README.md                            # este arquivo
│
├── include/control_node/
│   ├── pid_controller.hpp               # a lei de controle IDB+PID
│   ├── mg_motor_serial.hpp              # interface do driver RS485
│   ├── mot_reader.hpp                   # parser do .mot
│   └── trajectory_proc.hpp              # filtro 6 Hz + derivadas
│
├── src/
│   ├── control_node.cpp                 # nó de controle (loop tempo real)
│   ├── mg_motor_serial.cpp              # driver RS485 (protocolo K-Tech)
│   ├── trajectory_node.cpp              # nó publicador da trajetória
│   ├── mot_reader.cpp                   # implementação do parser
│   └── trajectory_proc.cpp             # implementação do processamento
│
├── config/
│   └── control.yaml                     # TODOS os parâmetros ajustáveis
│
├── launch/
│   └── control.launch.py                # sobe os dois nós juntos
│
└── data/
    └── subject01_walk1.mot              # arquivo de marcha (sua trajetória)
```

### 3.2 Tópicos e serviços ROS 2

| Nome | Tipo | Direção | Conteúdo |
|---|---|---|---|
| `/control/reference` | `Float64MultiArray` | trajectory → control | `[θref, θ̇ref, θ̈ref]` em rad |
| `/control/state` | `Float64MultiArray` | control → telemetria | `[θ, τ, erro, τ_est, temp]` |
| `/control/set_torque` | `SetBool` | usuário → control | liga/desliga o torque |

### 3.3 O fluxo dentro de cada nó

**`trajectory_node`** (a cada 5 ms):
1. (na inicialização) lê o `.mot`, filtra a 6 Hz, reamostra, calcula θ̇ e θ̈
2. publica o próximo ponto `[θref, θ̇ref, θ̈ref]` no `/control/reference`
3. ao chegar no fim, reinicia (se `loop: true`)

**`control_node`** (a cada 5 ms):
1. lê o ângulo atual do motor (comando `0x92`, multi-loop)
2. roda o PID → calcula o torque
3. envia o torque ao motor (comando `0xA1`)
4. publica a telemetria no `/control/state`
5. verifica a temperatura (corte de segurança)

---

## 4. Os parâmetros do motor e como afetam o código

Esta é a parte mais importante para entender por que o código é como é. Cada
número do datasheet do MG8008E-i9 tem um reflexo direto no software.

### 4.1 Tabela de parâmetros (do datasheet) → efeito no código

| Parâmetro do motor | Valor | Onde aparece | O que afeta |
|---|---|---|---|
| **Torque Constant (Kt)** | 2,0 N·m/A | `kt` | Conversão torque→corrente |
| **Max Current** | 12 A | `i_max` | Limite de corrente |
| **Max Torque** | 20 N·m | `tau_max` | Saturação do torque |
| **Gear Ratio** | 9:1 | `gear_ratio` | Escala ângulo/velocidade |
| **Encoder** | 16-bit | (multi-loop) | Leitura de ângulo |
| **CAN/RS485 baud** | 115200 | `baudrate` | Velocidade da serial |
| **Working Temp** | -20~+80 °C | `temp_limit` | Corte de segurança |

### 4.2 Detalhamento de cada um

**Torque Constant (Kt = 2,0 N·m/A)** — é a relação entre corrente e torque. O
motor não recebe "torque" diretamente; ele recebe um comando de **corrente**.
Então o código precisa converter:

```cpp
double current = torque_nm / cfg_.kt;   // torque desejado -> corrente
```

⚠️ **Sutileza da redução:** o datasheet lista Kt=2,0, que é o valor **no eixo
de saída** (depois da caixa de redução 9:1). O Kt do rotor seria 0,22 (= 2,0/9).
Como nosso controle atua no joelho (eixo de saída), usamos Kt=2,0 direto.

**Max Current (12 A)** — a corrente máxima que o driver entrega. O código nunca
manda mais que isso:

```cpp
if (current >  cfg_.i_max) current =  cfg_.i_max;
if (current < -cfg_.i_max) current = -cfg_.i_max;
```

**Max Torque (20 N·m)** — descoberto também na prática. Vira o `tau_max` que
satura a saída do PID. Mesmo que o controlador "queira" mandar 25 N·m num pico,
o código corta em 20:

```cpp
double tau_sat = std::clamp(tau_cmd, -cfg_.tau_max, cfg_.tau_max);
```

A simulação mostrou que essa saturação em 20 N·m não destrói o rastreio (RMS
fica ~0,19°), porque o feedforward fornece o grosso do torque de forma suave.

**Gear Ratio (9:1)** — esta é a mais sutil e importante. O motor tem uma caixa
de redução: o rotor gira **9 voltas** para o eixo de saída girar **1**. Isso
afeta DUAS coisas no código:

1. *Velocidade:* o motor reporta a velocidade do **rotor**; dividimos por 9 para
   obter a velocidade do **joelho**:
   ```cpp
   state.velocity = speed_rotor_rad_s / cfg_.gear_ratio;
   ```

2. *Ângulo:* usamos o comando `0x92` (multi-loop angle), que já retorna o ângulo
   acumulado do **eixo de saída** — então não precisamos multiplicar/dividir, o
   motor já entrega o ângulo certo do joelho.

> Se ignorássemos a redução, o ângulo lido estaria 9× errado e o controle
> divergiria completamente.

**Encoder (16-bit)** — a resolução do sensor de posição. Em vez de decodificar
os bits crus (que variam entre 14, 16 e 18-bit dependendo de qual encoder do
motor), usamos o comando `0x92` que entrega o ângulo já em **0,01°/LSB**,
padronizado. O código só converte para radianos:

```cpp
double multiloop_to_rad = 0.01 * M_PI / 180.0;   // 0,01 graus -> rad
```

**Baudrate (115200)** — a velocidade da comunicação serial. Tem uma
**consequência importante**: a 115200 baud, cada transação (mandar torque +
ler ângulo) leva ~3,4 ms. Por isso o loop de controle roda a **200 Hz (5 ms)**,
não a 1 kHz como na simulação. Isso está refletido no `update_rate_ms: 5`.

> Para taxas maiores, o motor suporta 1 Mbps — bastaria configurar no app
> LingLong e mudar `baudrate: 1000000` no YAML.

**Working Temperature (-20 a +80 °C)** — o motor opera até 80 °C. O código tem
um corte de segurança conservador em 70 °C, que desliga o torque
automaticamente se o motor esquentar:

```cpp
if (last_state_.temperature >= temp_limit_) {
  motor_->motorOff();   // desliga por segurança
}
```

### 4.3 Por que essa correspondência importa

Se você trocar de motor (outro modelo K-Tech, por exemplo), **só precisa mudar
o `control.yaml`** — o código não muda. Os parâmetros físicos estão todos
isolados em parâmetros ROS 2, exatamente para isso. Mudou o motor? Atualize
Kt, i_max, tau_max, gear_ratio e baudrate no YAML, e está pronto.

---

## 5. Os ganhos do PID

Os ganhos `Kp`, `Ki`, `Kd` vêm do projeto por **alocação de polos** feito no
TCC (não são tentativa e erro):

```yaml
kp: 190.6
ki: 884.1
kd: 16.07
```

Foram calculados para um amortecimento ζ=0,8 e frequência natural ωn=8 rad/s,
escolhida para que a banda do controlador (~1,3 Hz) cubra o espectro da marcha
humana (fundamental ~1 Hz). As fórmulas fechadas são:

```
Kd = D11*wn*(2*zeta + alpha) - b
Kp = D11*wn^2*(1 + 2*zeta*alpha)
Ki = D11*alpha*wn^3
```

Se quiser uma resposta mais rápida ou mais lenta, mude `wn` e recalcule (ou
ajuste os ganhos direto no YAML e observe o efeito).

O **feedforward** usa os parâmetros físicos da planta:

```yaml
D11: 0.2158    # inércia efetiva [kg.m^2]
mgd: 7.848     # torque gravitacional máximo [N.m]
b: 0.5         # atrito viscoso [N.m.s/rad]
use_feedforward: true   # true=IDB+PID ; false=PID puro
```

Coloque `use_feedforward: false` para testar o PID puro (vai rastrear pior — é
a comparação que valida a importância do feedforward).

---

## 6. Como subir e usar na Raspberry Pi

### 6.1 Subir o código para o GitHub (no seu PC)

```bash
cd control_node
git add -A
git commit -m "control_node RS485 serial"
git push
```

### 6.2 Acessar a Raspberry via RealVNC

Abra o **RealVNC Viewer** no PC do laboratório, conecte no IP da Pi (já
cadastrado) e use a senha do lab. Você verá a área de trabalho da Raspberry.

### 6.3 Clonar na Pi (desativando o proxy do lab)

```bash
# o proxy do lab bloqueia o GitHub: desative antes
unset http_proxy https_proxy
git config --global --unset http.proxy 2>/dev/null

cd ~/Documentos/Dev/UEFS/IC/sensorhub/src
git clone https://github.com/SEU_USUARIO/SEU_REPO.git control_node
```

### 6.4 Resolver permissão do Docker (uma vez só)

```bash
sudo usermod -aG docker $USER
# faça LOGOUT e LOGIN para valer; ou use sudo nos comandos docker
```

### 6.5 Dar acesso à porta serial

```bash
ls /dev/ttyUSB*                  # confirme que aparece /dev/ttyUSB0
sudo usermod -aG dialout $USER   # acesso à serial (logout/login depois)
```

### 6.6 Entrar no Docker e compilar

```bash
cd ~/Documentos/Dev/UEFS/IC/sensorhub/src/dev   # pasta COM o docker-compose.yaml
docker compose run --rm sensorhub
# (dentro do container agora)
cd /app/ros2_ws
colcon build --symlink-install --packages-select control_node
source install/setup.bash
```

Se a Pi travar na compilação (ela é limitada de RAM):
```bash
colcon build --symlink-install --executor sequential --packages-select control_node
```

### 6.7 Confirmar que o container vê a serial

```bash
ls /dev/ttyUSB*   # o docker-compose do Márcio deve mapear a porta
```

### 6.8 Rodar

```bash
ros2 launch control_node control.launch.py
```

Você verá:
```
[trajectory_node] subject01_walk1.mot: ... amostras
[control_node] iniciado: /dev/ttyUSB0 @ 115200 baud, 200 Hz, tau_max=20.0 N.m
[control_node] torque DESABILITADO. Chame /control/set_torque com data:true
```

### 6.9 Habilitar o torque (motor começa parado)

Em **outro terminal** (entre no mesmo container, `source install/setup.bash`):

```bash
ros2 service call /control/set_torque std_srvs/srv/SetBool "{data: true}"
```

⚠️ **Neste momento o motor começa a seguir a marcha.** Garanta que está preso e
ninguém está no caminho.

### 6.10 Ver a telemetria (opcional)

```bash
ros2 topic echo /control/state
# [theta_rad, tau_Nm, erro_rad, torque_estimado_Nm, temperatura_C]
```

### 6.11 Parar

```bash
ros2 service call /control/set_torque std_srvs/srv/SetBool "{data: false}"
# depois Ctrl-C no terminal do launch
```

---

## 7. Testando sem o motor

Você pode validar metade do sistema (a leitura do `.mot`) sem o motor ligado.
O `trajectory_node` não depende do hardware:

```bash
# terminal 1
ros2 run control_node trajectory_node --ros-args \
  -p mot_file:=/app/ros2_ws/src/control_node/data/subject01_walk1.mot

# terminal 2
ros2 topic echo /control/reference
```

Se aparecerem números `[θ, θ̇, θ̈]` rolando, a leitura e o processamento do
`.mot` funcionam. Faça isso ANTES de ligar o motor, para isolar problemas.

---

## 8. Calibração elétrica pendente

Há **um único parâmetro** que ainda precisa de calibração com o motor real:
`raw_max` (em `mg_motor_serial.hpp`, default 2000). Ele mapeia a corrente
máxima para o valor bruto do comando de torque enviado ao motor.

**Como calibrar:**
1. Monte o motor sob carga (a perna, ou um braço com peso conhecido).
2. Fonte em 24 V, limite de corrente ≥ 12 A.
3. Mande um torque conhecido e meça a corrente real (na fonte ou nas fases
   IA/IB/IC do app LingLong).
4. Ajuste `raw_max` até que o torque comandado bata com o medido
   (torque medido = corrente × Kt = corrente × 2,0).

A saturação em `tau_max=20 N·m` protege o motor mesmo antes da calibração.

> **Nota sobre testes sem carga:** com o motor solto na bancada, mandar torque
> baixo produz corrente quase nula (não há nada resistindo). Para ver torque e
> calibrar, o motor PRECISA estar contra uma carga.

---

## 9. Segurança

- O nó inicia com o torque **desabilitado**. Nada se move até você chamar o
  serviço `/control/set_torque`.
- A **saturação** em `tau_max=20 N·m` está ativa em todos os caminhos do código.
- **Corte por temperatura**: se o motor passar de 70 °C, o torque é desligado
  automaticamente.
- No encerramento (Ctrl-C), o destrutor **zera o torque e desliga o motor**.
- Recomenda-se um **botão de emergência físico** em série com a alimentação do
  motor, independente do software.

---

## 10. Solução de problemas

| Sintoma | Causa provável | Solução |
|---|---|---|
| `git clone` trava | proxy do lab | `unset http_proxy https_proxy` |
| `permission denied` (docker) | usuário fora do grupo docker | `sudo` ou `usermod -aG docker` |
| `no configuration file provided` | pasta errada do compose | `cd .../sensorhub/src/dev` |
| `/dev/ttyUSB0` não existe | conversor USB não reconhecido | verifique o cabo/driver USB-serial |
| `nao foi possivel abrir a porta serial` | sem permissão ou porta errada | `usermod -aG dialout` ou ajuste `port` |
| compilação trava a Pi | RAM insuficiente | `--executor sequential` |
| motor não se move / corrente zero | sem carga ou limite de corrente baixo | motor sob carga + fonte ≥ 12 A |
| ângulo lido errado (9× ou escala estranha) | redução não aplicada | confirme `gear_ratio: 9.0` |
| `colcon: command not found` | fora do container Docker | entre no Docker primeiro |

---

## Validação já realizada

- **Classe PID** (compilada e testada): degrau de 45° converge com erro nulo;
  saturação em 20 N·m correta.
- **Marcha 1 Hz** com IDB+PID e tau_max=20: RMS de regime ≈ 0,19°.
- **Protocolo serial** (frame 0x3E + checksum): conferido contra a captura real
  do app LingLong Motor Tool.
- **Parser do .mot**: lê a coluna, converte graus→rad, filtra a 6 Hz, calcula
  derivadas — validado contra valores teóricos (velocidade de pico bate com
  o esperado da senoide).

O comportamento do código espelha os modelos MATLAB/Simulink do TCC.