package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.interfaces.motor.MotorIO.MotorIOInputs;
import frc.lib.logger.LoggedTunableNumber;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final int id;
  private final Trigger coastButtonPressed;

  private final LoggedTunableNumber kPRollerMotor =
      new LoggedTunableNumber("Subsystems/Intake/RollerMotor/kP", IntakeConstants.ROLLER_KP);
  private final LoggedTunableNumber kIRollerMotor =
      new LoggedTunableNumber("Subsystems/Intake/RollerMotor/kI", IntakeConstants.ROLLER_KI);
  private final LoggedTunableNumber kDRollerMotor =
      new LoggedTunableNumber("Subsystems/Intake/RollerMotor/kD", IntakeConstants.ROLLER_KD);
  private final LoggedTunableNumber kSRollerMotor =
      new LoggedTunableNumber("Subsystems/Intake/RollerMotor/kS", IntakeConstants.ROLLER_KS);
  private final LoggedTunableNumber kVRollerMotor =
      new LoggedTunableNumber("Subsystems/Intake/RollerMotor/kV", IntakeConstants.ROLLER_KV);

  private final LoggedTunableNumber kPIntakeMotor =
      new LoggedTunableNumber("Subsystems/Intake/IntakeMotor/kP", IntakeConstants.INTAKE_KP);
  private final LoggedTunableNumber kIIntakeMotor =
      new LoggedTunableNumber("Subsystems/Intake/IntakeMotor/kI", IntakeConstants.INTAKE_KI);
  private final LoggedTunableNumber kDIntakeMotor =
      new LoggedTunableNumber("Subsystems/Intake/IntakeMotor/kD", IntakeConstants.INTAKE_KD);
  private final LoggedTunableNumber kFIntakeMotor =
      new LoggedTunableNumber("Subsystems/Intake/IntakeMotor/kF", IntakeConstants.INTAKE_KF);

  private LoggedTunableNumber currentLimitIntakeMotor =
      new LoggedTunableNumber(
          "Subsystems/Intake/CurrentLimitIntakeMotor", IntakeConstants.CURRENT_LIMIT_INTAKE_MOTOR);
  private LoggedTunableNumber currentLimitRollerMotor =
      new LoggedTunableNumber(
          "Subsystems/Intake/CurrentLimitRollerMotor", IntakeConstants.CURRENT_LIMIT_ROLLER_MOTOR);

  private LoggedTunableNumber voltageCompensationIntakeMotor =
      new LoggedTunableNumber(
          "Subsystems/Intake/VoltageCompensationIntakeMotor",
          IntakeConstants.VOLTAGE_COMPENSATION_INTAKE_MOTOR);

  public Intake(IntakeIO io) {
    this.io = io;
    id = hashCode();
    setName("Subsystems/Intake");
    io.configurePIDSVRollerMotor(
        kPRollerMotor.get(),
        kIRollerMotor.get(),
        kDRollerMotor.get(),
        kSRollerMotor.get(),
        kVRollerMotor.get());
    io.configurePIDFIntakeMotor(
        kPIntakeMotor.get(), kIIntakeMotor.get(), kDIntakeMotor.get(), kFIntakeMotor.get());
    ConstantsLogger.logConstants(IntakeConstants.class, getName());
    resetPosition(Rotations.of(IntakeConstants.INTAKE_START_POSITION));
    coastButtonPressed = new Trigger(() -> inputs.coastButtonPressed);
    coastButtonPressed
        .debounce(0.5)
        .onTrue(new InstantCommand(() -> io.setBrakeMode(false)).ignoringDisable(true))
        .onFalse(new InstantCommand(() -> io.setBrakeMode(true)).ignoringDisable(true));
  }

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    if (Constants.tuningMode) {
      if (kPRollerMotor.hasChanged(id)
          || kIRollerMotor.hasChanged(id)
          || kDRollerMotor.hasChanged(id)
          || kSRollerMotor.hasChanged(id)
          || kVRollerMotor.hasChanged(id)) {

        io.configurePIDSVRollerMotor(
            kPRollerMotor.get(),
            kIRollerMotor.get(),
            kDRollerMotor.get(),
            kSRollerMotor.get(),
            kVRollerMotor.get());
      }

      if (kPIntakeMotor.hasChanged(id)
          || kIIntakeMotor.hasChanged(id)
          || kDIntakeMotor.hasChanged(id)
          || kFIntakeMotor.hasChanged(id)) {

        io.configurePIDFIntakeMotor(
            kPIntakeMotor.get(), kIIntakeMotor.get(), kDIntakeMotor.get(), kFIntakeMotor.get());
      }
    }

    if (currentLimitIntakeMotor.hasChanged(hashCode())) {
      io.setCurrentLimitIntakeMotor(Amps.of(currentLimitIntakeMotor.get()));
    }

    if (currentLimitRollerMotor.hasChanged(hashCode())) {
      io.setCurrentLimitRollerMotor(Amps.of(currentLimitRollerMotor.get()));
    }

    if (voltageCompensationIntakeMotor.hasChanged(hashCode())) {
      io.setVoltageCompensationIntakeMotor(Volts.of(voltageCompensationIntakeMotor.get()));
    }
    PeriodicTimer.stop(getName());
  }

  public void resetPosition(Angle position) {
    io.resetPosition(position);
  }

  public void runVelocity(AngularVelocity velocity) {
    io.runVelocityRollerMotor(velocity);
  }

  public void runPosition(Angle position) {
    io.runPositionIntakeMotor(position);
  }

  public void runPercentOutputRollerMotor(double percentOutput) {
    io.runPercentOutputRollerMotor(percentOutput);
  }

  public void runPercentOutputIntakeMotor(double percentOutput) {
    io.runPercentOutputIntakeMotor(percentOutput);
  }

  public void stopRollerMotor() {
    io.stopRollerMotor();
  }

  public void stopIntakeMotor() {
    io.stopIntakeMotor();
  }

  public MotorIOInputs getRollerMotorInputs() {
    return inputs.rollerMotorInputs;
  }

  public MotorIOInputs getIntakeMotorInputs() {
    return inputs.intakeMotorInputs;
  }

  public InstantCommand setToMaxCurrent() {
    return new InstantCommand(
        () -> {
          io.setCurrentLimitIntakeMotor(Amps.of(60));
          io.setCurrentLimitRollerMotor(Amps.of(60));
        });
  }

  public InstantCommand setToNormalCurrent() {
    return new InstantCommand(
        () -> {
          io.setCurrentLimitIntakeMotor(Amps.of(currentLimitIntakeMotor.get()));
          io.setCurrentLimitRollerMotor(Amps.of(currentLimitRollerMotor.get()));
        });
  }
}
