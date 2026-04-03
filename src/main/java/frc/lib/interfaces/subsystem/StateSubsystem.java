package frc.lib.interfaces.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.fsm.StateMachine;
import frc.lib.util.ConstantsLogger;
import frc.lib.util.PeriodicTimer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public abstract class StateSubsystem<R extends Enum<R>, S extends Enum<S>, I extends LoggableInputs, T extends SubsystemIO<I>>
    extends SubsystemBase {

  protected final StateMachine<S> fsm;
  protected final I inputs;
  protected final T io;
  protected R currentRequest;

  public StateSubsystem(
      String name, I inputs, T io, Class<S> stateEnum, S initialState, R initialRequest) {
    setName(name);
    this.inputs = inputs;
    this.io = io;
    this.currentRequest = initialRequest;

    this.fsm = new StateMachine<>(name, stateEnum, initialState);
  }

    public StateSubsystem(
      String name,
      I inputs,
      T io,
      Class<S> stateEnum,
      S initialState,
      R initialRequest,
      Class<?> constantsClass) {
    setName(name);
    this.inputs = inputs;
    this.io = io;
    this.currentRequest = initialRequest;
    this.fsm = new StateMachine<>(name, stateEnum, initialState);

    ConstantsLogger.logConstants(constantsClass, name);
  }

  public void setRequest(R request) {
    this.currentRequest = request;
  }

  public R getRequest() {
    return currentRequest;
  }

  public S getState() {
    return fsm.getCurrentState();
  }

  public I getInputs() {
    return inputs;
  }

  protected boolean isRequest(R request) {
    return currentRequest == request;
  }

  protected boolean notInState(S state) {
      return getState() != state;
  }

  @Override
  public final void periodic() {
    PeriodicTimer.start(getName());

    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    fsm.update();

    sPeriodic();

    Logger.recordOutput(getName() + "/request", currentRequest.toString());

    PeriodicTimer.stop(getName());
  }

  protected void sPeriodic() {}

  public abstract boolean atGoal();
}
