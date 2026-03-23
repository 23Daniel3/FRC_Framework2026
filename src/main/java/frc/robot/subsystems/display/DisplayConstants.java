package frc.robot.subsystems.display;

import edu.wpi.first.wpilibj.SerialPort;

public final class DisplayConstants {
  public static final int BAUD_RATE = 9600;
  public static final SerialPort.Port PORT_USB = SerialPort.Port.kUSB;
  public static final SerialPort.Port PORT_MXP = SerialPort.Port.kMXP;

  private DisplayConstants() {}
}
