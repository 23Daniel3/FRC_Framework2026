package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.lib.util.security.CurrentSpikeDetector;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final CurrentSpikeDetector spikeDetector;

  private LoggedTunableNumber kp =
      new LoggedTunableNumber("Subsystems/Climber/P", ClimberConstants.kP);
  private LoggedTunableNumber ki =
      new LoggedTunableNumber("Subsystems/Climber/I", ClimberConstants.kI);
  private LoggedTunableNumber kd =
      new LoggedTunableNumber("Subsystems/Climber/D", ClimberConstants.kD);
  private LoggedTunableNumber kF =
      new LoggedTunableNumber("Subsystems/Climber/F", ClimberConstants.kF);

  public Climber(ClimberIO io) {
    this.io = io;
    setName("Subsystems/Climber");

    // Inicializa o detector (ajuste o tempo de 0.5s conforme necessário)
    this.spikeDetector =
        new CurrentSpikeDetector(
            ClimberConstants.MAX_SECURITY_CURRENT, ClimberConstants.THRESOLD_CURRENT_SPIKE);

    io.configurePIDF(
        ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, ClimberConstants.kF);
    ConstantsLogger.logConstants(ClimberConstants.class, getName());
    resetPosition();
    io.configureMaxOutput(0.7);
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());
    io.updateInputs(inputs);
    Logger.processInputs("Subsystems/Climber", inputs);

    if (kp.hasChanged(hashCode())
        || ki.hasChanged(hashCode())
        || kd.hasChanged(hashCode())
        || kF.hasChanged(hashCode())) {
      io.configurePIDF(kp.get(), ki.get(), kd.get(), kF.get());
    }

    // Atualiza o detector e para o motor se houver um pico persistente
    if (spikeDetector.update(inputs.motorInputs.current.in(Amps))) {
      stop();
      Logger.recordOutput("Subsystems/Climber/SpikeDetected", true);
    } else {
      Logger.recordOutput("Subsystems/Climber/SpikeDetected", false);
    }
    PeriodicTimer.stop(getName());
  }

  public void runVolts(Voltage volts) {
    // Impede o movimento se o detector estiver ativo
    if (spikeDetector.getAsBoolean()) {
      stop();
    } else {
      io.setVoltage(volts);
    }
  }

  public void runPercentOutput(double percentOutput) {
    if (spikeDetector.getAsBoolean()) {
      stop();
    } else {
      io.runPercentOutput(percentOutput);
    }
  }

  public void stop() {
    io.stop();
  }

  public void runPosition(Angle position) {
    // Substituída a verificação hardcoded de 45A pela lógica do detector
    if (spikeDetector.getAsBoolean()) {
      stop();
    } else {
      io.runPosition(position);
    }
  }

  public Angle getPosition() {
    return inputs.motorInputs.position;
  }

  public void resetPosition() {
    io.setOffset(Rotations.of(0));
  }

  public boolean isMotorConected() {
    return inputs.motorInputs.isConnected;
  }

  public MotorIOInputs getMotorInputs() {
    return inputs.motorInputs;
  }

  /** Retorna o detector para ser usado em gatilhos de comandos (Triggers) */
  public CurrentSpikeDetector getSpikeDetector() {
    return spikeDetector;
  }
}
