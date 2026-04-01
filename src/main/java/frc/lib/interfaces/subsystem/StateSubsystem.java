package frc.lib.interfaces.subsystem;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.interfaces.fsm.StateMachine;
import frc.lib.util.PeriodicTimer;

public abstract class StateSubsystem<R extends Enum<R>, S extends Enum<S>, I extends LoggableInputs> 
    extends SubsystemBase {
  
  protected final StateMachine<S> fsm;
  protected final I inputs;
  protected R currentRequest;

  public StateSubsystem(String name, I inputs, Class<S> stateEnum, S initialState, R initialRequest) {
    setName(name);
    this.inputs = inputs;
    this.currentRequest = initialRequest;
    
    this.fsm = new StateMachine<>(name, stateEnum, initialState);
  }

  public void setRequest(R request) {
    this.currentRequest = request;
  }

  public R getRequest() { return currentRequest; }
  public S getState() { return fsm.getCurrentState(); }

  protected abstract void updateInputs();

  @Override
  public void periodic() {
    PeriodicTimer.start(getName());
    
    updateInputs();
    Logger.processInputs(getName(), inputs);
    
    fsm.update();
    
    Logger.recordOutput(getName() + "/request", currentRequest.toString());
    
    PeriodicTimer.stop(getName());
  }

  public abstract boolean atGoal();
}