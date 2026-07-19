# Análise e Crítica — Base de Código FRC (TechMaker Swerve 2026)

Objetivo: avaliar a arquitetura atual (FSM, gerenciamento de subsistemas, comandos e superestrutura) e apontar o que precisa mudar para que o projeto vire um **template confiável** para todos os robôs futuros.

---

## 1. Visão geral — o que já está bom

A fundação é sólida e vale a pena preservar como espinha dorsal do template:

- **Camada IO no padrão AdvantageKit** (`SubsystemIO<I>`, `*IOInputsAutoLogged`, implementações Hardware/Sim/vazia para replay), com seleção por `Constants.currentMode`. É o padrão da comunidade (6328/Mechanical Advantage) e permite replay determinístico.
- **`MotorBase` unificado** com `MotorController`, `MotorConfig` fluente, 4 slots de PID/FF/SmartMotion tunáveis em runtime via `LoggedTunableNumber`, e implementações TalonFX/SparkMax/SparkFlex intercambiáveis. Isso é ouro para um template multi-fornecedor — trocar um NEO por um Kraken vira mudança de uma linha.
- **`StateMachine<S>` declarativa** com `onEnter/onUpdate/onExit`, transições locais e globais, e logging automático de estado/tempo/transições. A API fluente (`fsm.state(X).onEnter(...).transitionTo(Y, cond)`) é legível e fácil de ensinar a alunos novos.
- **Padrão Request → State** no `StateSubsystem`: o mundo externo só expressa *intenção* (enum de Request); a FSM interna decide *como* chegar lá; `atGoal()` fecha o contrato. Excelente separação para robótica.
- **`driveCore` + `RotationStrategy` + `TranslationModifier`**: composição por estratégia para o drivetrain (rotação manual, PID, heading por stick, aim em ponto/bola/hub; modificadores de sucção/repulsão de zona, throttle map, assistência de alvo). É o melhor pedaço de design do projeto — qualquer comportamento novo de teleop vira uma combinação, não um comando novo de 100 linhas.
- **Infra de suporte madura**: `LoggedTunableNumber/Map`, `ConstantsLogger` (loga constantes por reflexão), `PeriodicTimer`, zonas geométricas (`Polygon2d` etc.) com log de polígonos, `AllianceFlipUtil` abrangente, anti-tipping, detectores de corrente/motor travado.

O restante deste documento é crítica — mas o veredito geral é: **a arquitetura está certa; a execução tem bugs e inconsistências que um template não pode carregar.**

---

## 2. Bugs funcionais (corrigir antes de qualquer coisa)

### 2.1 O RPM calculado pelo SOTM nunca chega ao Shooter — *crítico*

`SuperStructure` mantém dois `ShotOnTheMoveCalculator` (hub e feed), calcula `ShotParameters` (ângulo + RPM) todo ciclo e usa o **ângulo** para mirar o drivetrain (`PIDRotation`, `JoystickDriveShooting`, `AimHub`). Porém:

- `Shooter.setVelocity(AngularVelocity)` **não é chamado em nenhum lugar do projeto** (grep completo). O campo `velocity` do Shooter permanece em `RPM.of(0.0)` para sempre.
- Portanto, `FLYWHEEL_RAMPING`/`SHOOTING` rodam o flywheel a **0 RPM**, e `isFlywheelReadyToKick()`/`atSetpoint` comparam contra 0.

Ou o robô real tem outro caminho que não está neste zip, ou o tiro "funciona" por coincidência de tolerâncias. Para o template, o contrato precisa ser explícito: a SuperStructure (ou o default command do Shooter) deve empurrar `getActiveShotParameters().rpm()` para o Shooter todo ciclo.

### 2.2 Setpoint aplicado apenas no `onEnter` impede SOTM contínuo

Mesmo que `setVelocity()` fosse chamado continuamente, os estados do Shooter só executam `io.controlFlywheel().runVelocity(velocity)` **no `onEnter`**. Enquanto o robô se move (shot-on-the-move!), o RPM alvo muda a cada ciclo, mas o hardware só recebe o valor congelado do momento da transição.

**Correção de template:** estados cujo setpoint é dinâmico devem aplicar o controle no `onUpdate`, não no `onEnter`. Regra prática para documentar no template: *`onEnter` para efeitos discretos (parar, mudar modo, agendar LED); `onUpdate` para setpoints que seguem um alvo.*

### 2.3 `getTimeInState()` está em microssegundos → `transitionAfter()` quebrado

`StateMachine` usa `Logger.getTimestamp()` do AdvantageKit, que retorna o timestamp da FPGA **em microssegundos**. Logo:

- `getTimeInState()` retorna µs, não segundos;
- `transitionAfter(double seconds, ...)` compara segundos contra microssegundos e dispara ~imediatamente;
- o log `FSM/<nome>/TimeInState` publica números na casa dos milhões.

Hoje ninguém chama `transitionAfter` (só a definição existe), então o bug está latente — exatamente o tipo de armadilha que explode no primeiro robô novo que usar o template. Correção: dividir por 1e6 (ou usar `Timer.getFPGATimestamp()`, que já é em segundos e é replay-safe quando usado via AdvantageKit).

### 2.4 Comparações exatas de ponto flutuante travam a FSM geral

Duas condições usam igualdade exata com zero:

- `Drivetrain.IsMoving()` → `getLinearVelocity() != 0`. Com ruído de encoder/odometria, isso é praticamente sempre `true`.
- `Intake` `STOPING → STOPPED` → `rollerMotorInputs.velocity.in(RPM) == 0`. Um roller em coast raramente reporta exatamente 0.0.

Consequência em cadeia: a transição `IDLEING → IDLE` da SuperStructure exige `intake.atGoal() && ... && !drivetrain.IsMoving()`. Com esses dois predicados ruidosos, o robô pode **ficar preso em IDLEING para sempre** (LEDs em chase ciano eternos são o sintoma visível). Corrigir com deadband (`< 0.05 m/s`, `< 20 RPM`), e adicionar ao template um utilitário `Deadband.isStopped(...)` para padronizar.

### 2.5 Limitadores de aceleração são código morto

`Drivetrain.applyLimitersRobotRelative(speeds)` retorna um `ChassisSpeeds` novo **com os mesmos valores** — não aplica nada. Enquanto isso, `xLimiter`, `yLimiter` e `hLimiter` (`DynamicSlewRateLimiter`) existem, têm seus limites atualizados no `periodic()`, mas nunca são invocados no caminho de controle. Ou seja: toda a camada de "security" de aceleração do drivetrain é decorativa. Decidir: ou aplicar de verdade (`xLimiter.calculate(speeds.vx)` etc.) ou remover — um template não pode conter uma função de segurança que parece existir mas não faz nada.

### 2.6 Conflito de escrita em `setMaxSpeed` (dois donos, sem arbitragem)

Dois atores escrevem `drivetrain.setMaxSpeed()` sem se conhecerem:

1. O `onEnter` de quase todos os estados da FSM geral (MAX_SPEED ou MAX_VELOCITY_TO_SHOOT);
2. O binding `rightTrigger(0.7)` do piloto (modo lento on/off).

O último a escrever vence. Se o piloto ativa o modo lento e depois qualquer transição de estado acontece, o modo lento é silenciosamente desfeito (e vice-versa). Para o template: o limite de velocidade deve ter **um único dono** (sugestão: a SuperStructure calcula `min(limitePorEstado, limitePorPiloto)`), ou virar um sistema de "speed constraints" empilháveis com prioridade.

### Observação: o triplo giro de frames em `driveCore` funciona, mas é ilegível

`driveCore` faz: `AllianceFlipUtil.apply(...)` (que internamente chama `fromRobotRelativeSpeeds` com heading possivelmente girado 180°) → `fromFieldRelativeSpeeds(...)` → `driveFieldRelative(...)` (que chama `fromFieldRelativeSpeeds` de novo). Fiz a álgebra: o resultado líquido é correto (drive field-relative com flip de aliança), mas são **três conversões de frame encadeadas para expressar uma**. Qualquer aluno que tentar modificar isso vai quebrar. Já `JoystickDriveShooting` faz o flip manualmente negando os sticks — uma terceira convenção. Padronizar em um único helper: `sticks → ChassisSpeeds field-relative já flipado → driveFieldRelative`.

---

## 3. Crítica da biblioteca de FSM (`lib/interfaces/fsm`)

### 3.1 Transições globais viraram "mapeamento de request" com guardas manuais frágeis

O padrão real de uso é sempre o mesmo:

```java
fsm.addGlobalTransition(GOING_SHOOT,
    () -> request == SHOOT
        && state != SHOOTING
        && state != SHOOTING_RECOVERY);
```

Problemas:

1. **Guardas redundantes:** o framework já checa `t.target != current`, então todo `notInState(TARGET)` é ruído (vários no Conveyor e Intake).
2. **Guardas de exclusão manuais e propensos a erro:** o autor precisa lembrar de listar *todos* os estados "já a caminho" do objetivo (SHOOTING, SHOOTING_RECOVERY...). Esquecer um causa loop de re-entrada: a transição global dispara, `onEnter` reseta o estado (para o flywheel, reagenda LED), e o progresso é perdido. É exatamente o tipo de bug que aparece com um mecanismo novo.
3. **É conceito de primeira classe disfarçado:** o que se quer dizer é "o request X é satisfeito pelo conjunto de estados {A, B, C}; se o request é X e não estou nesse conjunto, entre por A".

**Proposta para o template** — elevar isso à API:

```java
fsm.request(ShooterRequest.SHOOT)
   .entryState(FLYWHEEL_RAMPING)
   .satisfiedBy(FLYWHEEL_RAMPING, KICKER_RAMPING, SHOOTING);
```

Isso elimina os guardas manuais, torna `atGoal()` derivável automaticamente (request satisfeito ⇔ estado ∈ conjunto) e remove o switch repetitivo de `atGoal()` de cada subsistema (ver 4.2).

### 3.2 Uma transição por ciclo encadeia latência

`update()` executa no máximo um salto por chamada (bom — impede loops infinitos), mas cadeias tipo `IDLEING → IDLE` custam um ciclo extra (20 ms) por salto. Em cadeias de 2–3 estados intermediários isso soma 40–60 ms de latência estrutural. Aceitável, mas deve ser **documentado no template** e, idealmente, oferecer um `updateUntilStable(maxHops)` opcional para transições que são puramente lógicas (sem efeito de hardware entre elas).

### 3.3 Ordem de avaliação não documentada

Hoje: `onUpdate` → globais (na ordem de inserção) → locais (na ordem de inserção). Globais **preemptam** locais. Isso está correto para o uso atual (requests mandam), mas é conhecimento tribal — nada no código declara a prioridade. Num template, isso precisa estar em Javadoc do `StateMachine` e/ou virar explícito (`Transition.withPriority(...)`).

### 3.4 Faltas menores da lib

- `state()` chamado duas vezes para o mesmo enum silenciosamente sobrescreve a config anterior — deveria lançar exceção.
- Estados do enum sem registro são silenciosamente ignorados (`if (state == null) return`) — um `validateComplete()` chamado no fim do construtor do subsistema pegaria "esqueci de configurar SHOOTING" em bench test em vez de na competição.
- Não há hook de transição para o observador externo (útil p/ LEDs — ver 5.3).
- `forceState()` existe mas não é usado; se ficar, documentar que ele **não** respeita guardas (é a saída de emergência).

---

## 4. Crítica do padrão de gerenciamento de subsistemas

### 4.1 Dois caminhos de controle concorrentes — o mais grave estruturalmente

Existem **duas formas de comandar um subsistema**, e elas brigam:

1. **Via SuperStructure (a intencional):** default commands de Conveyor/Intake/Shooter fazem `subsystem.setRequest(superStructure.getXxxRequest())` **todo ciclo**.
2. **Via factories diretas:** `ConveyorCommands.run(conveyor)`, `ShooterCommands.shoot(shooter)` etc. fazem `setRequest` direto.

Como os default commands reescrevem o request a cada 20 ms assim que retomam, qualquer `runOnce` das factories diretas é **desfeito no ciclo seguinte**. Essas factories são, na prática, inutilizáveis em teleop — mas continuam no código convidando o próximo programador a usá-las (os bindings comentados do operador no `RobotContainer` fazem exatamente isso). Num template, isso é uma armadilha garantida.

**Decisão a tomar (e documentar):** escolher UM modelo.

- **Modelo A — SuperStructure como única fonte (recomendado):** a SuperStructure já tem referência a todos os subsistemas; os `onEnter` da FSM geral chamam `subsystem.setRequest(...)` diretamente. Isso **elimina os três default commands relay** (menos 1 ciclo de latência por hop na cadeia botão→SuperStructure→request→subsistema, que hoje pode chegar a 2–3 ciclos), e as factories diretas por subsistema são removidas ou movidas para um pacote `test/` de bench.
- **Modelo B — só comandos:** a SuperStructure não guarda referências; tudo flui por Commands com requirements. Mais "WPILib puro", porém abre mão da FSM central. Dado o investimento no modelo atual, A é o caminho natural.

### 4.2 `atGoal()` é boilerplate derivável

Todo subsistema implementa o mesmo switch `request → estado esperado`. Além de repetitivo, admite drift (adicionar um request e esquecer o case só aparece em runtime). Com a proposta 3.1 (`satisfiedBy`), `atGoal()` vira implementação final na classe base. O caso `NON_INTENTION → true` do Conveyor é um smell que confirma o problema: um request que significa "não tenho opinião" não deveria existir — se ninguém tem intenção, o request é `STOP` e ponto.

### 4.3 Estados quase duplicados e nomenclatura

- `RobotState` tem 12 estados, mas `IDLE/IDLEING` e `CLOSED/CLOSING` são pares "settling/settled" com efeitos quase idênticos (diferem em `IntakeRequest.STOP` vs `IN` e cor de LED). O padrão "GOING_X → X" se repete 5 vezes na FSM geral e mais vezes nos subsistemas. Vale abstrair: um construtor `settlingPair(GOING_X, X, cond)` no `StateMachine`, ou aceitar menos estados e usar `atGoal()` composto.
- Typos que vão se propagar para todos os robôs futuros se entrarem no template: `IDLEING` (→ `IDLING` ou `SETTLING`), `STOPING` (→ `STOPPING`), `AutoTrajetorys` (→ `AutoTrajectories` ou `Autos`), "Trajetorys".
- `Shooter.almostReadyToShoot()` não é usado; `Intake` tem duas transições globais para `GOING_OUT` que podiam ser uma com `request == OUT || request == COLLECT`.

### 4.4 Ergonomia do `StateSubsystem`

A assinatura genérica `StateSubsystem<R, S, I extends LoggableInputs, T extends SubsystemIO<I>>` com 6–7 argumentos de construtor é intimidadora para calouros — e o template vai ser lido por eles. Sugestões: manter os generics, mas fornecer um exemplo `ExampleSubsystem` exaustivamente comentado no template + um snippet/gerador (até um script Python simples que gera subsistema novo a partir do nome e dos enums — o tipo de coisa que economiza 30 min por mecanismo na temporada).

---

## 5. Crítica da SuperStructure

### 5.1 Papel híbrido e responsabilidades demais

Hoje a SuperStructure: (a) roda a FSM geral; (b) publica requests para 3 subsistemas via relay; (c) comanda o drivetrain **diretamente** (`setMaxSpeed`); (d) **agenda Commands de LED** de dentro do `onEnter`; (e) roda os dois calculadores SOTM; (f) loga PDH. São três estilos de coordenação diferentes coexistindo (requests, chamada direta, scheduling). Para o template, padronizar: a SuperStructure **coordena por requests/valores**, e efeitos periféricos (LED) viram observadores.

### 5.2 Parâmetros mortos e logging fora de lugar

- O construtor recebe `vision`, `driverControl` e `operatorControl` e não usa nenhum dos três — remover.
- `PowerDistribution` e seu logging não pertencem à SuperStructure; mover para `Robot`/um `PowerLogger` dedicado. No template, a SuperStructure deve ser o exemplo canônico de "coordenador limpo".

### 5.3 LEDs: de `schedule()` dentro do onEnter para observador

Agendar `LedCommands.*` dentro de `onEnter` funciona, mas: mistura o scheduler dentro do periodic de um subsistema, espalha decisões de UX pela FSM, e como `Led` não tem default command, um comando de LED "vaza" indefinidamente após o estado morrer. Mais limpo e mais template-friendly: um mapa declarativo `RobotState → efeito de LED` aplicado num único lugar (ou `Trigger`s sobre `getState()` no RobotContainer). A FSM fica só com lógica de robô; a "cara" do robô fica num arquivo só, fácil de retematizar por temporada.

### 5.4 Transição `SHOOTING → GOING_SHOOT` derruba o modo COLLECT_SHOOT

Em `COLLECT_SHOOTING`, perder o alinhamento transiciona para `GOING_SHOOT` (não `GOING_COLLECT_SHOOT`) — o intake para de coletar até a transição global do request COLLECT_SHOOT recapturar o estado (1 ciclo depois, re-executando `onEnter` de `GOING_COLLECT_SHOOT` e reagendando LED). Funciona por acidente graças à transição global, mas gera churn de onEnter/LED. Apontar para o estado "going" correto.

---

## 6. Comandos, factories e autônomo

### 6.1 Factories: padrão certo, conteúdo a podar

O padrão "factory estática por domínio" é bom e deve ficar no template. Mas: as factories diretas de subsistema são inúteis no modelo atual (ver 4.1); `SuperStructureCommands.manageRequests` como *default command* que traduz botões→request é uma boa ideia central — só formalizar a prioridade (o `close` vence tudo; `collect+shoot` = COLLECT_SHOOT; documentar essa tabela no código).

### 6.2 Autônomos

- `SuperStructureCommands.shoot(...).until(() -> false)`: `startEnd` já não termina sozinho; o `.until(() -> false)` é ruído que sugere incompreensão do ciclo de vida — remover (aparece 5×).
- Nos autos L/R, o **último passo é inalcançável**: o penúltimo (`Commands.parallel(AimHub, shoot.until(false))`) nunca termina, então o `shoot` final nunca roda. Inofensivo, mas confuso.
- `L_...` e `R_...` diferem apenas na string do path — parametrizar (`shootCollectShoot(String pathName)`).
- `sideChooser` é populado mas quase não é consumido; `WaitCommand(5)` como deadline de mira é constante mágica — mover para constants.
- Para o template, considerar migrar a seleção de autos para **PathPlanner named commands + um builder**, reduzindo esses SequentialGroups artesanais.

### 6.3 Restos da temporada anterior e higiene

- `AlignToReef`, `AlignToReefXConstants` etc. são do jogo 2025 (Reefscape) convivendo com hub/trench/bump (estilo 2022/2026). Num template, código de jogo específico deve viver em `frc.game`/`robot.commands.game` claramente separado do núcleo reutilizável — a separação `lib/` vs `robot/` vs `game/` já existe e é boa; falta disciplina para respeitá-la (`FieldConstants.Zones` referenciado com FQN dentro do Drivetrain, `Drivetrain` conhecendo trench/bump — isso é conhecimento de jogo dentro de um subsistema genérico; injete predicados de zona em vez de hardcodar).
- `MoveWithAutopilotAimHub.java.bkp` no repositório; blocos grandes de bindings comentados no `RobotContainer`; `// return Commands.none();` — tudo isso sai. Template limpo é template respeitado.

### 6.4 Aliança: cinco fontes da verdade

Hoje coexistem: `DriverStation.getAlliance()`, `AllianceSelector`, `AllianceManager`, `Constants.alliance` (fallback) e `Drivetrain.isBlue()`. `AllianceFlipUtil` usa uma, `JoystickDriveShooting` usa `AllianceManager`, `RobotContainer` usa `DriverStation.orElse(AllianceSelector)`. Consolidar em **um** serviço (`AllianceManager`) que encapsula o fallback, e todos os demais consomem dele. Divergência de aliança é o tipo de bug que só aparece na competição, quando o FMS conecta.

---

## 7. Roadmap sugerido para o template

**Fase 1 — Corrigir (bugs):**
1. Ligar `ShotParameters.rpm()` ao Shooter e mover setpoints dinâmicos para `onUpdate` (2.1, 2.2).
2. Corrigir unidades do `StateMachine` (µs→s) (2.3).
3. Deadbands em `IsMoving()` e `STOPING→STOPPED` (2.4).
4. Aplicar ou remover os slew rate limiters (2.5).
5. Arbitragem única de `maxSpeed` (2.6).

**Fase 2 — Consolidar (arquitetura):**
6. Escolher o Modelo A (SuperStructure escreve requests direto; remover default commands relay e factories diretas de subsistema) (4.1).
7. API `request().entryState().satisfiedBy()` na FSM + `atGoal()` automático + `validateComplete()` (3.1, 3.4, 4.2).
8. LED como observador declarativo de `RobotState` (5.3).
9. Unificar aliança no `AllianceManager` (6.4).
10. Simplificar o caminho de frames do `driveCore` (2.7).

**Fase 3 — Polir (template):**
11. Limpar restos de 2025, `.bkp`, código comentado, typos (`IDLING`, `STOPPING`, `AutoTrajectories`).
12. `ExampleSubsystem` comentado + gerador de subsistema.
13. Documentar no código: ordem de avaliação da FSM, regra onEnter vs onUpdate, tabela de prioridade de requests do piloto, convenção de frames.
14. Parametrizar autos e mover números mágicos para constants.

Com a Fase 1 e os itens 6–7 da Fase 2, o padrão Request→FSM→IO fica robusto o suficiente para ser a base de todos os robôs novos — o resto é qualidade de vida que paga dividendos a cada temporada.
