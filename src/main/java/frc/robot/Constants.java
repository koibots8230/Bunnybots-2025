package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.PIDGains;

public class Constants {

  public class RobotConstants {}

  public class ShooterConstants {
    public static final PIDGains SHOOTER_PID = new PIDGains.Builder().kp(0).build();
    public static final FeedforwardGains SHOOTER_FF = new FeedforwardGains.Builder().kv(0).build();

    public static final Current CURRENT_LIMIT = Amps.of(60);

    public static final int SHOOTER_MOTOR_ID = 0;
  }
}
