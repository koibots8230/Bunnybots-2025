package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public class Constants {

  public class RobotConstants {}

  public class IndexerConstants {

    public static final AngularVelocity INTAKING_SPEED = RPM.of(500);

    public static final PIDGains PID = new PIDGains.Builder().kp(0).build();

    public static final FeedforwardGains FEEDFORWARD = new FeedforwardGains.Builder().kv(0).build();

    public static final Distance MINIMUM_DISTANCE = Millimeters.of(0);

    public static final Current CURRENT_LIMMIT = Amps.of(60);

    public static final int LASER_CAN = 1;

    public static final int MOTOR_PORT = 1;
  public class ShooterConstants {
    public static final PIDGains SHOOTER_PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains SHOOTER_FF = new FeedforwardGains.Builder().kv(0).build();

    public static final Current CURRENT_LIMIT = Amps.of(60);

    public static final int SHOOTER_MOTOR_ID = 0;
  }
}
