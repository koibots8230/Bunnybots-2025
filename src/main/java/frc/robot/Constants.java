package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class Constants {

  public class RobotConstants {}

  public class IndexerConstants {
    public static final int MOTOR_PORT = 1;

    public static final double p = 1;

    public static final int LASER_CAN = 1;

    public static final Distance MINIMUM_DISTANCE = Millimeters.of(0);

    public static final AngularVelocity INTAKING_SPEED = RPM.of(500);
  }
}
