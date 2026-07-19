# Template FRC — Convenções e Arquitetura

Este documento é a referência rápida das regras do template. Leia antes de escrever código novo.
O exemplo canônico de subsistema está em `robot/subsystems/example/` (não instanciado); para gerar
um mecanismo novo use `python3 tools/new_subsystem.py NomeDoMecanismo [--vendor talonfx|sparkmax|sparkflex]`.

## Camadas

- **`lib/`** — código 100% reutilizável entre temporadas: FSM, IO de motores, utilitários, zonas,
  controladores. Não pode conhecer nada de `robot/` nem de `game/`.
- **`game/`** — regras e geometria do jogo do ano: campo, poses, janelas de tiro, `AllianceManager`.
- **`robot/`** — o robô do ano: subsistemas, superestrutura, comandos e bindings.

## Fluxo de controle (Modelo A)

```
piloto → SuperStructureCommands.manageRequests (default command; tabela de prioridade no Javadoc)
       → SuperStructure.setRequest(RobotRequest)
       → FSM geral: onEnter escreve DIRETAMENTE subsystem.setRequest(...) em cada subsistema
       → FSM do subsistema executa → io.controlXxx() → hardware
```

Regras:

1. **Só a SuperStructure escreve requests de subsistemas** em teleop/auto. Não crie Commands que
   chamem `subsystem.setRequest(...)` — eles brigariam com a FSM geral. Para bench test, escreva
   comandos temporários e não os commite.
2. Autônomos expressam intenção pelos factories de `SuperStructureCommands` (`shoot`, `collect`,
   `collectShooting`, ...), nunca por requests de subsistema.
3. LEDs são um **observador** do estado: o mapa `LedCommands.STATE_EFFECTS` é o único lugar que
   decide a cara do robô. A FSM não conhece LEDs.

## FSM (`lib/interfaces/fsm/StateMachine`)

- Ordem por ciclo: `onUpdate` do estado atual → transições **globais** (ordem de registro) →
  transições **locais** (ordem de registro). Globais têm prioridade. No máximo **uma** transição
  por ciclo (cadeias custam 20 ms por salto).
- `onEnter` = efeitos discretos (parar motor, trocar modo). `onUpdate` = setpoints que seguem alvo
  dinâmico (ex.: RPM do shot-on-the-move). Setpoint dinâmico no `onEnter` fica congelado.
- Requests são mapeados com `bindRequest(request, entrada, goal, intermediários...)` no
  `StateSubsystem` (ou `addRequestTransition` na FSM geral). Isso registra a transição global com
  as exclusões corretas e deriva `atGoal()` — **não** escreva switch manual de `atGoal`.
- Termine todo construtor de FSM com `fsm.validateComplete()`.
- Tempo: `getTimeInState()`/`transitionAfter()` estão em **segundos**.

## Sensores e predicados

- Nunca compare leitura de sensor com `== 0` / `!= 0`. Use deadband nomeado em Constants
  (`STOPPED_RPM_TOLERANCE`, `MOVING_DEADBAND_MPS`).

## Aliança — fonte única

- `AllianceSelector.getResolvedAlliance()` é a fonte canônica (FMS/DriverStation → chooser do
  dashboard → `Constants.alliance` como default do chooser).
- Código de `robot/`/`game/` consome via `AllianceManager` (`isBlue()`, `isRed()`, `shouldFlip()`);
  código de `lib/` consome direto do `AllianceSelector`. **Nunca** reimplemente o fallback com
  `DriverStation.getAlliance().orElse(...)`.

## Frames e drive

- Sticks do piloto são intenção **field-relative**. Na aliança vermelha o "campo do piloto" está
  girado 180° — isso equivale a negar X e Y (`AllianceFlipUtil.shouldFlip()`), como feito no
  `driveCore`. A única conversão field→robot acontece dentro de `driveFieldRelative`.
- Limite de velocidade tem **dois donos** com arbitragem por mínimo no Drivetrain:
  `setMaxSpeed` (estados da SuperStructure) e `setPilotMaxSpeed` (modo lento do piloto).
  Nunca escreva no limite do outro dono.

## Motores e IO

- Todo subsistema segue o padrão AdvantageKit: interface `XxxIO` com `@AutoLog` inputs +
  `XxxIOHardware` (+ Sim quando existir). `new XxxIO() {}` é o IO de replay.
- Motores só via `MotorIOSparkMax` / `MotorIOSparkFlex` / `MotorIOTalonFX` com `MotorConfig` —
  trocar fornecedor não pode vazar para fora do `IOHardware`.
- Constantes de subsistema (IDs, potências, tolerâncias, configs, enums Request/State) vivem no
  `XxxConstants` do próprio subsistema e são logadas via `ConstantsLogger`.

## Autônomos

- Sequências espelhadas L/R são **um** método parametrizado pelo nome do path.
- Factories da SuperStructure são `startEnd` e nunca terminam sozinhas: como último passo de um
  auto isso é intencional (roda até o fim do período); no meio de uma sequência, envolva em
  `deadline`/`withTimeout`.
- Tempos e números mágicos vão para `CommandConstants.AutoConstants`.

## Higiene

- Sem código comentado, sem arquivos `.bkp`, sem código do jogo anterior fora de `game/`.
- Código de temporadas passadas que valha como referência de padrão vai para `archive/` como
  `.java.txt` (fora do build) com uma linha no `archive/README.md`; história completa fica no git
  (uma tag por temporada).
- Nomes em inglês, enums em SCREAMING_CASE sem typos (é `IDLING`, `STOPPING`).
- Rode `./gradlew spotlessApply build` antes de todo commit.

## Testes e CI

- Testes de unidade em `src/test/java/frc/` cobrem a FSM, o padrão Request → State e os
  utilitários puros (interpolação, zonas, setpoint). Rode com `./gradlew test`.
- O workflow `.github/workflows/ci.yml` roda build + testes + Spotless em todo push/PR.
- Toda mudança na `lib/` (especialmente `StateMachine`/`StateSubsystem`) deve vir com teste —
  são as classes que todos os robôs futuros herdam.
