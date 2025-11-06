package frc.robot.Subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    final SparkMax shooterMotor;
    final SparkMaxConfig shooterMotorConfig;    
    SparkClosedLoopController shooterMotorController;
    public Shooter(){
        shooterMotor = new SparkMax(Constants.ShooterConstants.SHOOTER_MOTOR_ID, MotorType.kBrushless);
        shooterMotorConfig = new SparkMaxConfig();
        shooterMotorController = shooterMotor.getClosedLoopController();
        shooterMotorConfig.closedLoop.p(Constants.ShooterConstants.SHOOTER_P);
        shooterMotorConfig.closedLoop.velocityFF(Constants.ShooterConstants.SHOOTER_FF);
    }
    
    private void shoot(AngularVelocity shooterVelocity){
        shooterMotorController.setReference(shooterVelocity.in(RPM), ControlType.kVelocity);

    }
    public Command shootCommand(AngularVelocity shooterVelocity){
        return Commands.runOnce(() -> shoot(shooterVelocity), this);
    }
}
