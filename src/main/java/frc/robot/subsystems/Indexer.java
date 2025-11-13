package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import au.grapplerobotics.LaserCan;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

@Logged
public class Indexer extends SubsystemBase {

  @NotLogged private final SparkMax motor;

  @NotLogged private final LaserCan laserCAN;

  @NotLogged private final SparkMaxConfig config;

  @NotLogged private final SparkClosedLoopController closedLoopController;

  Current current;

  Voltage voltage;

  AngularVelocity velocity;

  Distance laserCANMeasurement;

  AngularVelocity setpoint;

  public Indexer() {
    motor = new SparkMax(IndexerConstants.MOTOR_PORT, MotorType.kBrushless);

    laserCAN = new LaserCan(IndexerConstants.LASER_CAN);

    config = new SparkMaxConfig();
    config.closedLoop.p(IndexerConstants.P);
    config.closedLoop.velocityFF(IndexerConstants.FF);
    config.inverted(false);
    config.smartCurrentLimit((int) IndexerConstants.CURRENT_LIMMIT.in(Amps));

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    closedLoopController = motor.getClosedLoopController();

    setpoint = RPM.of(0);
  }

  @Override
  public void periodic() {
    current = Amps.of(motor.get());
    voltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    velocity = RPM.of(motor.getEncoder().getVelocity());

    if (laserCAN.getMeasurement() != null) {
      laserCANMeasurement = Millimeters.of(laserCAN.getMeasurement().distance_mm);
    }
  }

  public void simulationPeriodic() {
    velocity = setpoint;
  }

  public boolean seePiece() {
    if (laserCAN.getMeasurement() != null) {
      return laserCAN.getMeasurement().distance_mm
          < IndexerConstants.MINIMUM_DISTANCE.in(Millimeters);
    } else {
      return false;
    }
  }

  private void setSpeed(AngularVelocity speed) {
    closedLoopController.setReference(speed.in(RPM), ControlType.kVelocity);
    setpoint = speed;
  }

  public Command setSpeedCommand(AngularVelocity speed) {
    return Commands.runOnce(() -> setSpeed(speed), this);
  }
}
