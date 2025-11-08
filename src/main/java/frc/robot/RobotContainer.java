// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {

  @NotLogged private final XboxController controller;

  private final Swerve swerve;

  private boolean isBlue;

  public RobotContainer(boolean isReal) {

    swerve = new Swerve(isReal);


    controller = new XboxController(0);

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public void setAlliance() {
    isBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.setIsBlue(isBlue);
  }
}
