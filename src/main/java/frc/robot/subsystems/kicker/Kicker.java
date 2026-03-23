package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Kicker extends SubsystemBase {

  private final KickerIO io;
  private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();
  private final int id;

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Subsystems/Kicker//kP", KickerConstants.KP);
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Subsystems/Kicker//kI", KickerConstants.KI);
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Subsystems/Kicker//kD", KickerConstants.KD);

  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Subsystems/Kicker/kS", KickerConstants.KS);
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Subsystems/Kicker/kV", KickerConstants.KV);
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Subsystems/Kicker/kA", KickerConstants.KA);

  private LoggedTunableNumber currentLimit =
      new LoggedTunableNumber("Subsystems/Kicker/CurrentLimit", KickerConstants.CURRENT_LIMIT);
  private LoggedTunableNumber voltageCompensation =
      new LoggedTunableNumber(
          "Subsystems/Kicker/VoltageCompensation", KickerConstants.VOLTAGE_COMPENSATION);

  private SimpleMotorFeedforward feedforward;

  public Kicker(KickerIO io) {
    this.io = io;
    id = hashCode();
    setName("Subsystems/Kicker");
    io.configurePID(kP.get(), kI.get(), kD.get());
    ConstantsLogger.logConstants(KickerConstants.class, getName());

    feedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
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
        feedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get());
      }
    }

    if (currentLimit.hasChanged(hashCode())) {
      io.setCurrentLimit(Amps.of(currentLimit.get()));
    }

    if (voltageCompensation.hasChanged(hashCode())) {
      io.setVoltageCompensation(Volts.of(voltageCompensation.get()));
    }

    PeriodicTimer.stop(getName());
  }

  public void runVelocity(AngularVelocity velocity) {
    double ffVoltageValue = feedforward.calculate(velocity.in(RadiansPerSecond));

    io.runVelocity(velocity, Volts.of(ffVoltageValue));
  }

  public void runPercentOutput(double percentOutput) {
    io.runPercentOutput(percentOutput);
  }

  public boolean hasElement() {
    return inputs.isSensorActive;
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
