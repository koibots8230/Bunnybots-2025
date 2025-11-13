// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Shooter;

@Logged
public class RobotContainer {

  @NotLogged private final XboxController controller;
  private final Shooter shooter;

  public RobotContainer() {
    shooter = new Shooter();
    controller = new XboxController(0);

    configureBindings();
  }

  private void configureBindings() {
    Trigger test = new Trigger(() -> controller.getAButton());
    test.onTrue(shooter.shootWithRPMOf(RPM.of(1000)));
    test.onFalse(shooter.shootWithRPMOf(RPM.of(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
