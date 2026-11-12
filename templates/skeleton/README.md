# Esqueleto de robô (Ciprix)

O menor robô completo no padrão do framework: um subsistema (`Roller`), a `SuperStructure` com a
máquina de estados geral, os comandos do piloto e o backup do copiloto. Use como ponto de partida
de um robô novo.

```
src/main/java/frc/robot/
  Robot.java                     AdvantageKit + configuração da Ciprix
  RobotContainer.java            subsistemas, botões, autônomo
  Constants.java                 modo, RobotRequest, RobotState, campo
  commands/SuperStructureCommands.java   botões do piloto -> RobotRequest
  subsystems/superstructure/SuperStructure.java   FSM geral
  subsystems/roller/             um subsistema: Constants, IO, IOTalonFX, IOSim, Roller
```

## Começar um robô novo

1. Copie esta pasta para fora do repositório (ou para `examples/<seu-robo>`).
2. Em `.wpilib/wpilib_preferences.json`, troque `teamNumber` pelo número da equipe.
3. Em `settings.gradle`, troque `rootProject.name`.
4. Abra a pasta no VS Code da WPILib. `Simulate Robot Code` já funciona: o roller simulado gira
   com o bumper esquerdo do piloto (controle 0) e inverte com o direito.

Fora deste repositório não existe a pasta `ciprix/` ao lado, então o Gradle baixa a versão
publicada indicada em `vendordeps/Ciprix*.json`.

## Adicionar um subsistema

Siga o `Roller` como modelo, sempre nesta ordem:

1. `XxxConstants`: ids, configurações de motor (`MotorConfig`), `enum XxxRequest` e
   `enum XxxState`.
2. `XxxIO`: os inputs com `@AutoLog` e um `controlYyy()` por motor, devolvendo
   `MotorControllerNone` por padrão (é o que roda no replay).
3. `XxxIOTalonFX` (ou `XxxIOSparkMax`, ...) e `XxxIOSim`, com a mesma configuração e o mesmo nome
   de motor, para os logs da simulação e do robô baterem.
4. `Xxx extends StateSubsystem<...>` com `@AutoCommandFactory`: um `fsm.state(...)` por estado,
   um `bindRequest(...)` por request e `fsm.validateComplete()` no fim.
5. Na `SuperStructure`: um campo `xxxRequest`, o getter `getXxxRequest()` (o comando gerado lê
   esse nome) e o request de cada estado geral no `onEnter`.
6. No `RobotContainer`: `xxx.setDefaultCommand(XxxCommands.defaultCommand(superStructure, xxx))`
   e, para o copiloto, `XxxCommands.<request>(xxx)` nos botões de backup.

O guia completo do padrão está em [`docs/arquitetura.md`](../../docs/arquitetura.md).
