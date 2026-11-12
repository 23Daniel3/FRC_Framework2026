# Simulação

A simulação usa a mesma configuração e os mesmos nomes de motor do robô, então máquinas de
estado, ganhos, alertas e logs se comportam igual. `Simulate Robot Code` no VS Code (ou
`./gradlew simulateJava`) abre a simulação; habilite o robô na janela da simulação.

## Motores com física

`MotorIOSim` imita o controlador de verdade (malha de velocidade e posição, Motion Magic/Smart
Motion, kS/kV/kA/kG, limite de corrente, un-jam, saúde) em cima de um modelo físico do mecanismo:

```java
private final MotorIOSim flywheel =
    MotorIOSim.talonFX(
            "Shooter/Leader",
            ShooterConstants.MOTOR_LEADER_CONFIG,
            SimMechanism.flywheel(DCMotor.getKrakenX60(2), 1.0, 0.004).withFriction(0.03, 0.0))
        .withFollowers(1);
```

| Mecanismo | Fábrica |
| --- | --- |
| Roller, esteira | `SimMechanism.roller(motor, redução, inércia)` |
| Flywheel | `SimMechanism.flywheel(motor, redução, inércia)` |
| Braço, pivot | `SimMechanism.arm(motor, redução, massaKg, comprimentoM)`, com gravidade |
| Elevador | `SimMechanism.elevator(motor, redução, massaKg, raioDoTamborM)`, com gravidade |

Ajustes: `withFriction(coulombNm, viscoso)`, `withLimits(minRot, maxRot)` (batentes),
`withStartPosition(rot)`, `withGravity(...)` e `withLoad(...)` (uma peça presa, por exemplo).
`MotorIOSim.talonFX(...)` usa as unidades e os ganhos do Talon; `MotorIOSim.spark(...)`, os do
SPARK (kV em V/RPM, como no REV Hardware Client).

Para testar a reação do robô a problemas: `setConnected(false)`, `setFaults("...")` e
`setTemperatureCelsius(95)` disparam os mesmos alertas do robô real.

## Bateria

Cada motor simulado soma a sua corrente em `SimBattery`, que derruba a tensão como uma bateria de
verdade. É isso que faz o [gerenciamento de energia](hardware-energia-e-pit.md#bateria-baixa) agir
na simulação também.

## Replay

Com `Constants.simMode = Mode.REPLAY`, o robô roda de novo um log do AdvantageKit: as IOs ficam
vazias e os inputs vêm do log. Serve para testar uma mudança de lógica (um filtro de visão, uma
transição da FSM) com os dados de uma partida real e comparar no AdvantageScope.

## Robô 3D

As peças móveis do CAD seguem os mecanismos (`RobotVisualizer` e `MechanismPoses`). O passo a passo
está em
[`examples/season-2026-rebuilt/advantagescope/README.md`](../examples/season-2026-rebuilt/advantagescope/README.md).

## Testes sem robô

`ciprix/core/src/test` tem os testes da biblioteca. A maioria roda sem a HAL, com o
`TestEnvironment` controlando o relógio, o estado habilitado e a bateria. É o jeito mais rápido de
provar que uma regra nova funciona antes de levar ao robô.
