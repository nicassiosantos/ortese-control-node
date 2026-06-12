# Roteiro de Testes Progressivos

Valide o sistema do **mais simples ao mais complexo**. Cada teste só deve ser
feito depois que o anterior passou. Isso isola problemas: se algo falhar, você
sabe exatamente em qual camada está.

> **Segurança em todos os testes:** motor preso na bancada, fonte 24 V com
> limite de corrente adequado, e a mão perto do botão de desligar. Comece com
> torques e velocidades BAIXOS.

---

## Teste 0 — Protocolo offline (sem motor)

Confirma que os frames gerados pelo código batem com o protocolo do motor.
Não precisa de motor nem de fonte.

```bash
ros2 run control_node mostra_protocolo
```

Compare os frames impressos com a captura do app LingLong. O frame de Motor ON
deve ser `3E 88 01 00 C7`. **Passou se** os bytes batem.

---

## Teste 1 — Comunicação (motor responde?)

Confirma que o motor responde aos comandos. Já validado (você viu TX e RX).

```bash
# com debug_frames: true no control.yaml
ros2 launch control_node control.launch.py
# noutro terminal:
ros2 service call /control/set_torque std_srvs/srv/SetBool "{data: true}"
```

**Passou se** aparece `[RX]` depois do `[TX]` no terminal.

---

## Teste 2 — Torque fixo direto (valida escala e sinal)

Manda um torque CONSTANTE direto no motor, sem PID. É o teste mais importante
para entender a resposta física e calibrar.

```bash
# NAO rode o control_node junto (disputaria a porta).
ros2 run control_node test_torque_node --ros-args \
    -p torque_nm:=0.5 -p duration_s:=3.0
```

Observe a tabela impressa (ângulo, velocidade, corrente, temperatura). Aumente
o torque aos poucos: 0.5 → 1.0 → 2.0 N·m.

**O que verificar:**
- A **corrente** sobe quando você aumenta o torque? (deve subir)
- O motor gira/faz força no sentido esperado? (confirma o SINAL)
- A corrente medida bate com `torque / Kt`? Ex: 2 N·m / 2.0 = 1 A esperado.
  Se a corrente real for muito diferente, o `raw_max` precisa de ajuste.

> **Este teste calibra o `raw_max`:** se você manda 2 N·m (espera 1 A) mas a
> corrente real é 3 A, a escala está 3x alta — ajuste `raw_max` proporcional.

---

## Teste 3 — Manter ângulo fixo (degrau do PID)

Agora COM o PID. O controlador deve segurar um ângulo fixo.

```bash
# terminal 1
ros2 launch control_node control.launch.py
# (mas SEM o trajectory_node publicando; use o test_node como referencia)
```

Melhor: rode o control_node sozinho e o test_node como referência:

```bash
# terminal 1: so o control_node
ros2 run control_node control_node --ros-args \
    --params-file install/control_node/share/control_node/config/control.yaml
# terminal 2: referencia de angulo fixo
ros2 run control_node test_node --ros-args -p mode:=hold -p angle_deg:=-20.0
# terminal 3: habilita
ros2 service call /control/set_torque std_srvs/srv/SetBool "{data: true}"
```

**Passou se** o motor vai para -20° e SEGURA sem tremer. Se tremer aqui, o
problema é ganho/calibração (não a trajetória).

---

## Teste 4 — Senoide lenta (trajetória suave conhecida)

Uma referência senoidal lenta e previsível. Bem mais fácil que a marcha.

```bash
# terminal 2 (no lugar do hold):
ros2 run control_node test_node --ros-args \
    -p mode:=sine -p amp_deg:=10.0 -p freq_hz:=0.2 -p offset_deg:=-20.0
```

Isso faz o joelho oscilar suavemente entre -30° e -10°, a 0.2 Hz (um ciclo a
cada 5 s). Bem lento e seguro.

**Vá aumentando a dificuldade aos poucos:**
- `freq_hz:=0.2` → bem lento (comece aqui)
- `freq_hz:=0.5` → moderado
- `freq_hz:=1.0` → velocidade de marcha

**Passou se** o motor segue a senoide suavemente, sem tremer.

---

## Teste 5 — Marcha real (o .mot completo)

Só depois que a senoide a 1 Hz funcionar bem.

```bash
ros2 launch control_node control.launch.py
ros2 service call /control/set_torque std_srvs/srv/SetBool "{data: true}"
```

---

## Gravando dados em qualquer teste

Em paralelo a qualquer teste acima, grave a telemetria:

```bash
ros2 run control_node logger_node --ros-args -p out_file:=/tmp/teste.csv
# ... roda o teste alguns segundos ...
# Ctrl-C no logger para fechar
python3 scripts/plotar_telemetria.py /tmp/teste.csv --salvar figuras/
```

O **espectro do torque** (terceira figura) mostra se há tremor e em que
frequência — essencial para diagnosticar problemas.

---

## Tabela de diagnóstico

| Onde falha | O que indica |
|---|---|
| Teste 0 | frames errados → revisar protocolo |
| Teste 1 | sem RX → baudrate, fonte ou fiação |
| Teste 2 corrente não sobe | motor sem carga, ou sinal/escala errados |
| Teste 2 corrente errada | calibrar `raw_max` |
| Teste 3 treme parado | ganho alto ou `raw_max` alto |
| Teste 4 treme na senoide | ganho do PID p/ a taxa atual (200 Hz) |
| Teste 5 treme só na marcha | derivadas da trajetória muito agressivas |