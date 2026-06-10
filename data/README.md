# Pasta de trajetórias (.mot)

Coloque aqui o arquivo `.mot` do OpenSim com a marcha que o motor deve seguir.

## Onde colocar

```
control_node/data/subject01_walk1.mot
```

O `trajectory_node` lê a coluna `knee_angle_r` deste arquivo por padrão.

## Qual arquivo usar

É a saída da **Inverse Kinematics Tool** do OpenSim — o mesmo
`subject01_walk1.mot` que você usou nos testes do MATLAB. Ele contém a coluna
`knee_angle_r` com o ângulo do joelho ao longo da marcha.

## Como apontar para outro arquivo

Por padrão o launch procura `data/subject01_walk1.mot`. Para usar outro:

```bash
ros2 launch control_node control.launch.py mot_file:=/caminho/para/sua_marcha.mot
```

## Formato esperado

Arquivo .mot padrão do OpenSim:
```
nome_do_movimento
nRows=...	nColumns=...
inDegrees=yes
endheader
time	knee_angle_r	...
0.000	-5.23	...
0.010	-5.31	...
...
```

O nó detecta automaticamente se os ângulos estão em graus (`inDegrees=yes`) e
converte para radianos. Filtra a 6 Hz, reamostra para a taxa do controlador e
deriva velocidade e aceleração antes de publicar.
