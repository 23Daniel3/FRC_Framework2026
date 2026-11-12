# Hardware, energia e pit

## Alertas de hardware e a tela piscando

Todo motor lido pelo código (e cada câmera da `AprilTagVision`) entra no `HardwareHealthMonitor`,
que publica:

| Chave (`/AdvantageKit/RealOutputs/...`) | O que é |
| --- | --- |
| `Hardware/Blink` | Alterna a 2,5 Hz enquanto algo está errado; fica falso com tudo certo |
| `Hardware/Healthy` | Verdadeiro com tudo conectado, sem falhas e sem problemas |
| `Hardware/Disconnected` | Nomes dos dispositivos que não respondem |
| `Hardware/Problems` | Uma linha por falha ou problema, como `Intake/Roller: Over temperature` |

Para a tela do piloto piscar em vermelho: no Elastic, um **Boolean Box** ligado a
`Hardware/Blink`, com vermelho para verdadeiro e a cor do fundo para falso. Ao lado, uma lista com
`Hardware/Problems` diz o que é. `HardwareHealthMonitor.blink()` também serve para um efeito de LED.

Motores que o código nunca lê (seguidores, por exemplo) não são julgados, e nos primeiros 5 s
depois de ligar nada conta como desconectado.

## Un-jam automático

```java
new MotorConfig().withStallReversal(Amps.of(30), 0.2, 0.3, 0.5)
```

Se o motor está sendo comandado, passa de 30 A e quase não gira por 0,2 s, ele inverte a 50% por
0,3 s e depois volta sozinho ao último comando do código. Enquanto inverte, os comandos do código
são guardados, não perdidos; um `stop()` para na hora. `inputs.stalled` e `inputs.unjamming`
aparecem no log.

## Bateria baixa

O `PowerManager` olha a bateria filtrada e, abaixo de 9,5 V, reduz o limite de corrente dos
motores de prioridade `NORMAL` e `LOW` e suaviza a aceleração do drivetrain; abaixo de 8 V corta
mais. Volta um degrau por vez quando a bateria fica 0,5 V acima do limite por 0,5 s.

| Prioridade | Normal | Abaixo de 9,5 V | Abaixo de 8 V |
| --- | --- | --- | --- |
| `HIGH` (padrão) | 100% | 100% | 100% |
| `NORMAL` | 100% | 75% | 50% |
| `LOW` | 100% | 50% | 25% |

```java
new MotorConfig().currentLimit(Amps.of(35)).powerPriority(PowerPriority.LOW)
```

No robô da equipe, o conveyor e o roller do intake são `LOW`; o shooter e o drivetrain não são
reduzidos (o drivetrain só tem a aceleração suavizada). No log: `Power/Level` e
`Power/FilteredVoltage`.

## Autoteste no pit

Cada subsistema declara, junto da máquina de estados, quais motores podem girar livres por um
instante:

```java
selfTest()
    .motor("Roller", io::controlRollerMotor, () -> inputs.rollerMotorInputs)
    .output(0.3)
    .currentBetween(Amps.of(0.3), Amps.of(25))
    .mustTurn(RotationsPerSecond.of(5));
```

Com o robô habilitado, os botões `SelfTest/<subsistema>` e `SelfTest/All` no dashboard giram cada
motor e conferem que ele responde, puxa uma corrente razoável e gira de verdade. Pega motor ou fio
desconectado, acoplamento quebrado, encoder morto e mecanismo travando. Durante o teste a máquina
de estados do subsistema fica pausada e, no fim, recomeça do estado inicial. O resultado sai em
`SelfTest/<subsistema>/<motor>` e uma falha vira alerta. Não use em braço ou elevador, que bateriam
no batente.

## Checklist da fila

Com o robô desabilitado, o `PreMatchChecklist` mostra o que ainda falta antes da partida:

```java
PreMatchChecklist.addDefaults(); // bateria >= 12,4 V, dispositivos conectados, sem falhas
PreMatchChecklist.add("Aliança definida", () -> DriverStation.getAlliance().isPresent());
PreMatchChecklist.add("Visão localizou o robô", vision::hasRecentPose);
```

No Elastic: um Boolean Box em `PreMatch/Ready` e uma lista em `PreMatch/Pending`. Um alerta de
aviso lista as pendências. Quando o robô é habilitado, o checklist para e o alerta some.
