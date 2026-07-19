# FRC Framework 2026

**Idioma:** **Português** | [English](./README.en.md)

Framework de robô FRC em Java (WPILib + GradleRIO + CTRE Phoenix 6 + PathPlanner + AdvantageKit), organizado para acelerar desenvolvimento de subsistemas com arquitetura **Request → State (FSM)**.

> Este README é o guia principal para uso no GitHub. Para convenções internas detalhadas, veja também `src/main/java/frc/TEMPLATE.md`.

---

## Visão geral da arquitetura

O projeto separa responsabilidades em 3 camadas:

- `src/main/java/frc/lib/` → infraestrutura reutilizável (FSM, utilitários, controle, zonas, wrappers).
- `src/main/java/frc/game/` → regras e geometria do jogo/campo (poses, zonas, aliança).
- `src/main/java/frc/robot/` → implementação do robô da temporada (subsystems, commands, factories, bindings).

### Fluxo de controle (teleop/auto)

1. Entradas do piloto vão para `SuperStructureCommands.manageRequests(...)`.
2. Esse comando escreve um `RobotRequest` na `SuperStructure`.
3. A FSM geral da `SuperStructure` decide o estado global.
4. No `onEnter` de cada estado global, a `SuperStructure` escreve requests dos subsistemas (`setRequest(...)`).
5. Cada subsistema executa sua própria FSM e aplica no hardware via camada IO.

Esse modelo evita conflito entre comandos paralelos e centraliza a intenção do robô.

---

## Estrutura do projeto (resumo)

```text
src/
  main/
    java/frc/
      lib/
      game/
      robot/
        commands/
        factories/
        subsystems/
          drivetrain/
          superstructure/
          intake/
          conveyor/
          shooter/
          led/
          vision/
          example/
      tools/
        new_subsystem.py
  test/
    java/frc/
      lib/
```

---

## Como os subsistemas funcionam

O padrão base é `StateSubsystem<Request, State, Inputs, IO>`.

### Contrato do padrão

- **Entrada externa:** `setRequest(Request)`
- **Estado interno:** FSM declarativa (`StateMachine`)
- **Saída:** chamadas de controle no IO (`io.controlXxx()...`)
- **Critério de conclusão:** `atGoal()` derivado via `bindRequest(...)`

### Boas práticas adotadas

- Use `onEnter` para efeito discreto (ex.: parar motor, trocar modo).
- Use `onUpdate` para setpoint dinâmico (ex.: RPM que varia com shot-on-the-move).
- Sempre finalize construtor com `fsm.validateComplete()`.
- Evite comparar sensor com `== 0`; prefira tolerância/deadband.

### Exemplo canônico

Use `src/main/java/frc/robot/subsystems/example/` como referência completa de:

- `XxxConstants`
- `XxxIO` (`@AutoLog` inputs)
- `XxxIOHardware`
- `XxxSubsystem` com FSM + `bindRequest`

---

## Drivetrain swerve (CTRE Phoenix 6)

O swerve é implementado em `src/main/java/frc/robot/subsystems/drivetrain/Drivetrain.java` e estende `TunerSwerveDrivetrain`.

### Principais responsabilidades

- Controle field-relative/robot-relative.
- Configuração de `AutoBuilder` (PathPlanner).
- Limitação de aceleração e anti-tipping.
- SysId (translation, steer, rotation).
- Integração com aliança para flip de referências.

### Estratégia de comando de drive

A base está em `src/main/java/frc/robot/factories/DrivetrainCommands.java`, com um núcleo `driveCore(...)` que:

- aplica modificadores de translação,
- resolve estratégia de rotação,
- aplica flip de campo por aliança,
- envia `ChassisSpeeds` para o drivetrain.

Isso permite compor comandos como:

- joystick normal,
- throttle map,
- heading hold,
- aim em ponto/alvo,
- repulsão/sucção de zonas,
- pathfinding para pose.

### Limites de velocidade (dois donos)

No drivetrain existe arbitragem por mínimo entre:

- limite do **estado global** (`setMaxSpeed` via SuperStructure),
- limite do **piloto** (`setPilotMaxSpeed`, modo lento).

Assim um não sobrescreve o outro.

---

## SuperStructure (orquestração do robô)

Arquivo: `src/main/java/frc/robot/subsystems/superstructure/SuperStructure.java`.

### O que ela faz

- Mantém a FSM geral de `RobotState`.
- Traduz `RobotRequest` em requests de subsistemas.
- Coordena modos como coletar, atirar, coletar+atirar, fechar.
- Ajusta limite de velocidade do drivetrain por estado.
- Calcula parâmetros ativos de tiro (shot-on-the-move).

### Integração com LEDs e auto

- LEDs seguem estado via comando observador (`followRobotState`).
- Autos usam factories da SuperStructure (`shoot`, `collect`, `collectShooting`, etc.), sem escrever request direto de subsistema.

---

## Comandos e factories

- `src/main/java/frc/robot/commands/` contém comandos específicos (drive alignment, autos, etc.).
- `src/main/java/frc/robot/factories/` concentra fábricas de comandos por subsistema/domínio.

### Recomendação do projeto

- Use factories para padronizar criação de comandos.
- Evite lógica duplicada de binding entre comandos diferentes.
- Em teleop, preserve a regra: **a intenção global entra pela SuperStructure**.

---

## Testes automatizados

Os testes estão em `src/test/java/frc/` com foco em núcleo lógico (sem hardware real).

### Suítes atuais (resumo)

- `StateMachineTest`
  - prioridade global > local,
  - no máximo uma transição por ciclo,
  - `transitionAfter` em segundos,
  - `forceState`, `validateComplete`, tempo por estado.
- `StateSubsystemTest`
  - `bindRequest` e `atGoal()` derivado,
  - não reentrada indevida,
  - troca de request no meio do caminho.
- `SetpointTrackerTest`, `LinearInterpolationTest`, `Polygon2dTest`
  - utilitários matemáticos e geométricos da `lib/`.

### Rodar testes e build (PowerShell)

```powershell
./gradlew test
./gradlew build
```

> O `build` já inclui compilação + testes.

---

## Como gerar novos subsistemas

Existe um gerador automático:

- Script: `src/main/java/frc/tools/new_subsystem.py`
- Base usada: `src/main/java/frc/robot/subsystems/example/`

### Exemplo de uso (raiz do repo)

```powershell
python src/main/java/frc/tools/new_subsystem.py Climber
python src/main/java/frc/tools/new_subsystem.py ArmPivot --vendor talonfx
```

Também funciona com `sparkmax` e `sparkflex`.

### Depois de gerar

1. Ajuste constantes e IDs em `XxxConstants`.
2. Modele enums `Request` e `State`.
3. Configure FSM + `bindRequest(...)` no subsistema.
4. Instancie no `RobotContainer` por modo (`REAL`, `SIM`, `REPLAY`).
5. Conecte requests na FSM da `SuperStructure` (se fizer parte do fluxo global).

---

## Como criar novos comandos

### 1) Defina o objetivo

- Comando local de subsistema?
- Comando global (via `RobotRequest`)?
- Comando para auto/path?

### 2) Escolha o lugar certo

- Comandos específicos: `src/main/java/frc/robot/commands/...`
- Factory reutilizável: `src/main/java/frc/robot/factories/...`

### 3) Padrões recomendados

- Para intenção global: prefira factory em `SuperStructureCommands`.
- Para rotina contínua: `Commands.run(...)` com `requirements` corretos.
- Para liga/desliga com cleanup: `Commands.startEnd(...)`.
- Nomeie comandos (`withName`) para telemetria clara.

### 4) Integre no `RobotContainer`

- binding de botão,
- default command,
- trigger contextual (disabled/autonomous/teleop).

---

## Setup rápido de desenvolvimento

Pré-requisitos:

- JDK 17 (WPILib 2026)
- Gradle Wrapper (já no repositório)
- VS Code + extensões FRC/WPILib

### Comandos úteis (PowerShell)

```powershell
./gradlew spotlessApply
./gradlew test
./gradlew build
```

---

## CI e qualidade

- O projeto usa formatação automática com Spotless (Google Java Format).
- `build` depende de `spotlessApply` no `build.gradle`.
- Consulte `README-CI-TESTES.md` para detalhes de CI/testes e cobertura atual.

---

## Dicas de contribuição

- Mantenha regras de arquitetura do `TEMPLATE.md`.
- Evite “atalhos” que furem a SuperStructure.
- Sempre que alterar `lib/interfaces/fsm` ou infraestrutura central, adicione/atualize testes.

---

## Licença

Consulte `WPILib-License.md` e as licenças dos vendors em `vendordeps/`.
