package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@Logged
public class Shooter extends SubsystemBase {

  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final SparkClosedLoopController motorController;
  private Voltage voltage;
  private AngularVelocity velocity;
  private Current current;
  private AngularVelocity setpoint;

  public Shooter() {
    motor = new SparkMax(ShooterConstants.MOTOR_PORT, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop.p(ShooterConstants.PID.kp);
    motorConfig.closedLoop.velocityFF(ShooterConstants.FEEDFORWARD.kv);

    motorConfig.smartCurrentLimit((int) ShooterConstants.CURRENT_LIMIT.in(Amps));

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorController = motor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    voltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    velocity = RPM.of(motor.getEncoder().getVelocity());
    current = Amps.of(motor.getOutputCurrent());
  }

  private void setVelocity(AngularVelocity velocity) {
    motorController.setReference(velocity.in(RPM), ControlType.kVelocity);
    setpoint = velocity;
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> setVelocity(velocity), this);
  }

  @Override
  public void simulationPeriodic() {
    velocity = setpoint;
  }
}
