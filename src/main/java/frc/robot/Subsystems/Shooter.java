package frc.robot.Subsystems;

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
import frc.robot.Constants;

@Logged
public class Shooter extends SubsystemBase {

  private final SparkMax shooterMotor;
  private final SparkMaxConfig shooterMotorConfig;
  private final SparkClosedLoopController shooterMotorController;
  private Voltage shooterVoltage;
  private AngularVelocity shooterVelocity;
  private Current shooterCurrent;

  public Shooter() {
    shooterMotor = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
    shooterMotorConfig = new SparkMaxConfig();
    shooterMotorConfig.closedLoop.p(Constants.ShooterConstants.SHOOTER_P);
    shooterMotorConfig.closedLoop.velocityFF(Constants.ShooterConstants.SHOOTER_FF);
    shooterMotor.configure(
        shooterMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooterMotorController = shooterMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    shooterVoltage = Volts.of(shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage());
    shooterVelocity = RPM.of(shooterMotor.getEncoder().getVelocity());
    shooterCurrent = Amps.of(shooterMotor.getOutputCurrent());
  }

  private void shoot(AngularVelocity shooterVelocity) {
    shooterMotorController.setReference(shooterVelocity.in(RPM), ControlType.kVelocity);
  }

  public Command shootCommand(AngularVelocity shooterVelocity) {
    return Commands.runOnce(() -> shoot(shooterVelocity), this);
  }
}
