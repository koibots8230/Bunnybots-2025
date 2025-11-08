package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

@Logged
public class SwerveModule {
  @NotLogged private final SparkFlex driveMotor;
  @NotLogged private final SparkMax turnMotor;

  @NotLogged private final SparkMaxConfig turnConfig;
  @NotLogged private final SparkFlexConfig driveConfig;

  @NotLogged private final AbsoluteEncoder turnEncoder;
  @NotLogged private final RelativeEncoder driveEncoder;

  @NotLogged private final SparkClosedLoopController turnController;
  @NotLogged private final SparkClosedLoopController driveController;

  @NotLogged private final SimpleMotorFeedforward turnFeedforward;

  @NotLogged private final TrapezoidProfile turnProfile;

  private TrapezoidProfile.State turnGoalState;
  private TrapezoidProfile.State turnSetpointState;

  private final Rotation2d offset;

  private Rotation2d turnSetpoint;
  private LinearVelocity driveSetpoint;

  private Distance drivePosition;
  private Rotation2d turnPosition;
  private LinearVelocity driveVelocity;
  private AngularVelocity turnVelocity;

  private Voltage turnVoltage;
  private Voltage driveVoltage;
  private Current turnCurrent;
  private Current driveCurrent;

  public SwerveModule(int driveID, int turnID) {

    if (driveID == SwerveConstants.FRONT_LEFT_DRIVE_ID) {
      offset = SwerveConstants.OFFSETS[0];
    } else if (driveID == SwerveConstants.FRONT_RIGHT_DRIVE_ID) {
      offset = SwerveConstants.OFFSETS[1];
    } else if (driveID == SwerveConstants.BACK_LEFT_DRIVE_ID) {
      offset = SwerveConstants.OFFSETS[2];
    } else {
      offset = SwerveConstants.OFFSETS[3];
    }

    turnProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                SwerveConstants.MAX_TURN_VELOCITY.in(RadiansPerSecond),
                SwerveConstants.MAX_TURN_ACCELRATION.in(RadiansPerSecondPerSecond)));

    turnGoalState = new TrapezoidProfile.State(0, 0);
    turnSetpointState = new TrapezoidProfile.State(0, 0);

    turnMotor = new SparkMax(turnID, MotorType.kBrushless);
    driveMotor = new SparkFlex(driveID, MotorType.kBrushless);

    turnConfig = new SparkMaxConfig();

    turnConfig.idleMode(IdleMode.kBrake);

    turnConfig
        .closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(SwerveConstants.TURN_PID.kp, SwerveConstants.TURN_PID.ki, SwerveConstants.TURN_PID.kd)
        .positionWrappingEnabled(true)
        .positionWrappingInputRange(-Math.PI, Math.PI);

    turnConfig.smartCurrentLimit((int) SwerveConstants.TURN_CURRENT_LIMIT.in(Amps));

    turnConfig.absoluteEncoder.positionConversionFactor(SwerveConstants.TURN_CONVERSION_FACTOR);
    turnConfig.absoluteEncoder.velocityConversionFactor(
        SwerveConstants.TURN_CONVERSION_FACTOR / 60);
    turnConfig.absoluteEncoder.inverted(true);

    driveConfig = new SparkFlexConfig();

    driveConfig.closedLoop.pidf(
        SwerveConstants.DRIVE_PID.kp,
        SwerveConstants.DRIVE_PID.ki,
        SwerveConstants.DRIVE_PID.kd,
        SwerveConstants.DRIVE_FEEDFORWARD.kv);

    driveConfig.idleMode(IdleMode.kBrake);

    driveConfig.smartCurrentLimit((int) SwerveConstants.DRIVE_CURRENT_LIMIT.in(Amps));

    driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_CONVERSION_FACTOR);
    driveConfig.encoder.velocityConversionFactor(SwerveConstants.DRIVE_CONVERSION_FACTOR / 60.0);

    turnMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    turnEncoder = turnMotor.getAbsoluteEncoder();
    turnController = turnMotor.getClosedLoopController();

    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getClosedLoopController();

    turnFeedforward =
        new SimpleMotorFeedforward(
            SwerveConstants.TURN_FEEDFORWARD.ks, SwerveConstants.TURN_FEEDFORWARD.kv);
  }

  public void setState(SwerveModuleState swerveModuleState) {
    swerveModuleState.optimize(
        Rotation2d.fromRadians(MathUtil.angleModulus(turnPosition.getRadians())));

    swerveModuleState.speedMetersPerSecond *= swerveModuleState.angle.minus(turnPosition).getCos();

    driveController.setReference(
        swerveModuleState.speedMetersPerSecond, SparkBase.ControlType.kVelocity);

    driveSetpoint = MetersPerSecond.of(swerveModuleState.speedMetersPerSecond);
    turnSetpoint = swerveModuleState.angle;
  }

  public void periodic() {
    driveCurrent = Amps.of(driveMotor.getOutputCurrent());
    turnCurrent = Amps.of(turnMotor.getOutputCurrent());

    driveVoltage = Volts.of(driveMotor.getAppliedOutput() * driveMotor.getBusVoltage());
    turnVoltage = Volts.of(turnMotor.getAppliedOutput() * turnMotor.getBusVoltage());

    drivePosition = Meters.of(driveEncoder.getPosition());
    turnPosition = Rotation2d.fromRadians(turnEncoder.getPosition() - offset.getRadians());

    driveVelocity = MetersPerSecond.of(driveEncoder.getVelocity());
    turnVelocity = RadiansPerSecond.of(turnEncoder.getVelocity());

    turnGoalState =
        new TrapezoidProfile.State(
            MathUtil.angleModulus(turnSetpoint.getRadians()) + offset.getRadians(), 0);

    turnSetpointState =
        turnProfile.calculate(
            RobotConstants.ROBOT_CLOCK_SPEED.in(Seconds), turnSetpointState, turnGoalState);

    turnController.setReference(
        turnSetpointState.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        turnFeedforward.calculate(turnSetpointState.velocity));
  }

  public void simulationPeriodic() {
    drivePosition = drivePosition.plus(driveSetpoint.times(RobotConstants.ROBOT_CLOCK_SPEED));
    turnPosition = turnSetpoint;
    driveVelocity = driveSetpoint;
  }

  public SwerveModuleState getModuleState() {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), Rotation2d.fromRadians(turnEncoder.getPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), Rotation2d.fromRadians(turnEncoder.getPosition()));
  }
}
