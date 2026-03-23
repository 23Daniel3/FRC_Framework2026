package frc.robot.subsystems.conveyor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import org.littletonrobotics.junction.Logger;

public class Conveyor extends SubsystemBase {

  private final ConveyorIO io;
  private final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();
  private LoggedTunableNumber currentLimit =
      new LoggedTunableNumber("Subsystems/Conveyor/CurrentLimit", ConveyorConstants.CURRENT_LIMIT);
  private LoggedTunableNumber voltageCompensation =
      new LoggedTunableNumber(
          "Subsystems/Conveyor/VoltageCompensation", ConveyorConstants.VOLTAGE_COMPENSATION);

  public Conveyor(ConveyorIO io) {
    this.io = io;
    setName("Subsystems/Conveyor");
    ConstantsLogger.logConstants(ConveyorConstants.class, getName());
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    if (currentLimit.hasChanged(hashCode())) {
      io.setCurrentLimit(Amps.of(currentLimit.get()));
    }

    if (voltageCompensation.hasChanged(hashCode())) {
      io.setVoltageCompensation(Volts.of(voltageCompensation.get()));
    }
    PeriodicTimer.stop(getName());
  }

  public void runPercentOutput(double percentOutput) {
    io.runPercentOutput(percentOutput);
  }

  public void stop() {
    io.stop();
  }

  public MotorIOInputs getMotorInputs() {
    return inputs.motorInputs;
  }

  public InstantCommand setToMaxCurrent() {
    return new InstantCommand(
        () -> {
          io.setCurrentLimit(Amps.of(60));
        });
  }

  public InstantCommand setToNormalCurrent() {
    return new InstantCommand(
        () -> {
          io.setCurrentLimit(Amps.of(currentLimit.get()));
        });
  }
}
