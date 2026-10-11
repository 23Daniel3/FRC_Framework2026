package frc.lib.util.security;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class LockedMotorDetector implements BooleanSupplier {
  private final double timeThresholdSeconds;
  private final Timer stallTimer;
  private Trigger cachedTrigger;
  private boolean lastValue = false;
  private double minVelocity = Double.POSITIVE_INFINITY;

  public LockedMotorDetector(double timeThresholdSeconds, double minVelocity) {
    this.timeThresholdSeconds = timeThresholdSeconds;
    this.stallTimer = new Timer();
    this.minVelocity = minVelocity;
  }

  public boolean update(boolean shouldBeMoving, double actualVelocity) {
    if (DriverStation.isDisabled()) {
      stallTimer.stop();
      stallTimer.reset();
      lastValue = false;
      return false;
    }

    if (shouldBeMoving && Math.abs(actualVelocity) < Math.abs(minVelocity)) {
      stallTimer.start();
      lastValue = stallTimer.hasElapsed(timeThresholdSeconds);
      return lastValue;
    } else {
      stallTimer.stop();
      stallTimer.reset();
      lastValue = false;
      return false;
    }
  }

  @Override
  public boolean getAsBoolean() {
    return lastValue;
  }

  public Trigger asTrigger() {
    if (cachedTrigger == null) {
      cachedTrigger = new Trigger(this);
    }
    return cachedTrigger;
  }
}
