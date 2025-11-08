// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {

  private final Swerve swerve;

  private boolean isBlue;

  private final SendableChooser<Command> autoChooser;

  public RobotContainer(boolean isReal) {

    swerve = new Swerve(isReal);

    autoChooser = new SendableChooser<>();

    setupAutos();

    configureBindings();
  }

  private void configureBindings() {}

  private void setupAutos() {
    autoChooser.setDefaultOption("Nothing", Commands.none());

    autoChooser.addOption("Test", swerve.autoDriveCommand(Meters.of(2), MetersPerSecond.of(3), Rotation2d.kZero));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setAlliance() {
    isBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.setIsBlue(isBlue);
  }
}
