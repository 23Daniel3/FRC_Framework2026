package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private final int id;

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Subsystems/Flywheel/kP", FlywheelConstants.KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Subsystems/Flywheel/kI", FlywheelConstants.KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Subsystems/Flywheel/kD", FlywheelConstants.KD);

  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Subsystems/Flywheel/kS", FlywheelConstants.KS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Subsystems/Flywheel/kV", FlywheelConstants.KV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Subsystems/Flywheel/kA", FlywheelConstants.KA);

  private final LoggedTunableNumber currentLimit =
      new LoggedTunableNumber("Subsystems/Flywheel/CurrentLimit", FlywheelConstants.CURRENT_LIMIT);

  public Flywheel(FlywheelIO io) {
    this.io = io;
    id = hashCode();
    setName("Subsystems/Flywheel");

    io.configurePID(kP.get(), kI.get(), kD.get());
    io.configureKSVA(kS.get(), kV.get(), kA.get());
    ConstantsLogger.logConstants(FlywheelConstants.class);
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    if (Constants.tuningMode) {
      if (kP.hasChanged(id) || kI.hasChanged(id) || kD.hasChanged(id)) {
        io.configurePID(kP.get(), kI.get(), kD.get());
      }
      if (kS.hasChanged(id) || kV.hasChanged(id) || kA.hasChanged(id)) {
        io.configureKSVA(kS.get(), kV.get(), kA.get());
      }
    }

    if (currentLimit.hasChanged(hashCode())) {
      io.setCurrentLimit(Amps.of(currentLimit.get()));
    }

    PeriodicTimer.stop(getName());
  }

  public void runVelocity(AngularVelocity velocity) {
    io.runVelocity(velocity);
  }

  public void runVoltage(Voltage voltage) {
    io.runVoltage(voltage);
  }

  public void runPercentOutput(double percentOutput) {
    io.runPercentOutput(percentOutput);
  }

  public void stop() {
    io.stop();
  }

  public MotorIOInputs getLeaderInputs() {
    return inputs.leaderInputs;
  }

  public MotorIOInputs getFollowerInputs() {
    return inputs.followerInputs;
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
