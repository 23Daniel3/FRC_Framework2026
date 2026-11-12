# Migração para a WPILib 2027

> Baseado no changelog oficial da 2027 ainda em **alpha** (Alpha 8, página atualizada em
> 30/10/2026). Antes de migrar, confira a versão final em
> [New for 2027](https://docs.wpilib.org/en/latest/docs/yearly-overview/yearly-changelog.html) e
> [Removed features](https://docs.wpilib.org/en/latest/docs/yearly-overview/removed-features.html).

## O que não muda

O padrão continua igual: piloto escreve requests, a SuperStructure decide o estado do robô, cada
subsistema tem a sua máquina de estados, o copiloto sobrepõe um subsistema enquanto segura o botão
e o hardware só é acessado pela IO. A máquina de estados da Ciprix não depende do agendador de
comandos, e o `StateSubsystem` continua sendo um `SubsystemBase` do command-based "v2", que segue
existindo na 2027. A migração é de nomes e APIs, não de arquitetura.

## O que muda na 2027 (e onde nos afeta)

| Mudança | Onde aparece aqui |
| --- | --- |
| roboRIO substituído pelo **Systemcore**, com vários barramentos CAN | Dispositivos CAN passam a receber um `CANPort`; as IOs da CTRE e da REV seguem o que os fornecedores lançarem para 2027 |
| **Java 25** e Gradle 9.4.1 | `sourceCompatibility` em `ciprix/build.gradle` e nos `build.gradle` dos projetos |
| Pacotes de `edu.wpi.first` para `org.wpilib` | Todos os arquivos; o importador do VS Code tenta trocar sozinho |
| `DriverStation` dividido em `MatchState` e `RobotState` | O nosso `Constants.RobotState` passa a ter o mesmo nome de uma classe da WPILib: renomeie para `SuperState` na migração (`RobotContainer` importa `wpilibj.*`) |
| SmartDashboard, `SendableChooser` e `Sendable` trocados por Telemetry e Tunables; Shuffleboard removido | Botões `SelfTest/...`, `LoggedDashboardChooser` (depende do AdvantageKit 2027), `PersistentAlert` |
| Timestamps inteiros em **nanossegundos** | `VisionIOLimelight` (`getFPGATime()` e timestamps do NetworkTables em microssegundos) |
| `Rotation2d.getDegrees()`/`getRadians()` devolvem o ângulo "embrulhado" | Conferir contas que dependem de ângulo contínuo (há `ContinuousUnwrapper` para isso) |
| `MathUtil.clamp` sai, usar `Math.clamp`; `Pose.exp(twist)` vira `plus(twist.exp())` | 6 arquivos da biblioteca (limitadores de aceleração, anti-tombamento, `ThrottleMap`, `SimMechanism`) e o `Drivetrain` do robô |
| `MotorController.set()` vira `setThrottle()`; controladores PWM vão para uma biblioteca de drivers | IOs PWM da biblioteca (`interfaces/motor/pwm`) |
| AprilTag e CameraServer viram vendordeps; `AprilTagFields` entra numa classe `Fields` | `VisionConstants` e o exemplo de `VisionFilter.field(...)` |
| Jackson trocado por Avaje Jsonb | `LimelightHelpers` do robô usa Jackson: trocar pela versão 2027 do arquivo ou pela `VisionIOLimelight` da biblioteca, que não precisa dele |
| Modo Test vira **Utility**; `robotInit()` sai (já usamos o construtor) | `Robot.java` |
| Contador, servo, SPI, giroscópios analógicos e outros saem do Systemcore | Nada aqui usa |

## Commands v3

A 2027 traz um command-based novo ("v3") com máquina de estados declarativa e uma interface
`Mechanism`, documentado lado a lado com o v2. Para a temporada 2027, o caminho seguro é continuar
no v2. Na offseason, vale avaliar um `StateSubsystem` montado sobre um `Mechanism` do v3 com a
mesma API (`fsm`, `bindRequest`, `override`, `selfTest`), para o código do robô não mudar nada.

## Ordem sugerida

1. Uma branch `wpilib-2027`, com o VS Code da WPILib 2027.
2. `ciprix/core` primeiro: trocar a versão do GradleRIO, deixar o importador renomear os pacotes,
   ajustar a tabela acima e rodar os testes (a maioria roda sem a HAL).
3. `ciprix/ctre`, `ciprix/rev` e `ciprix/phoenix5` quando a CTRE e a REV publicarem as versões
   2027 (o Phoenix 5 pode não ter versão para o Systemcore).
4. `templates/skeleton`, que é pequeno e mostra rápido o que falta.
5. O robô da equipe: renomear `RobotState` para `SuperState`, trocar o `LimelightHelpers`, conferir
   a simulação inteira e o replay de um log de 2026.
6. Só então ligar no Systemcore, com o robô suspenso, seguindo o checklist de
   [mudanças no robô real](mudancas-no-robo-real.md#para-conferir-na-primeira-ligada).
