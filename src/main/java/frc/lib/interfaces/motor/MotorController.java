package frc.lib.interfaces.motor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface MotorController {
    void setBrakeMode(boolean enabled);
    void setOffset(Angle offset);

    void runVoltage(Voltage volts);
    void runPercentOutput(double percent);

    void runVelocity(AngularVelocity velocity);
    void runPosition(Angle position);
    void runSmartPosition(Angle position);

    void runVelocity(AngularVelocity velocity, int slot);
    void runPosition(Angle position, int slot);
    void runSmartPosition(Angle position, int slot);

    void stop();

    void setCurrentLimit(Current current);
}