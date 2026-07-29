package frc.lib.util;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PeriodicSystem extends SubsystemBase {

  public PeriodicSystem() {
    super();
  }

  public PeriodicSystem(String name) {
    super(name);
  }

  @Override
  public abstract void periodic();

  @Override
  @Deprecated
  public final void setDefaultCommand(Command defaultCommand) {
    throw new UnsupportedOperationException("PeriodicSystem não suporta Comandos Padrão.");
  }

  @Override
  @Deprecated
  public final Command getDefaultCommand() {
    return null;
  }

  @Override
  @Deprecated
  public final Command getCurrentCommand() {
    return null;
  }

  @Override
  @Deprecated
  public final void setName(String name) {
    super.setName(name);
  }

  @Override
  @Deprecated
  public final void setSubsystem(String subsystem) {
    super.setSubsystem(subsystem);
  }

  @Override
  public final void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PeriodicSystem");
  }
}
