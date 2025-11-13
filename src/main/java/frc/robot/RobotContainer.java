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
import frc.robot.subsystems.Indexer;

@Logged
public class RobotContainer {

  @NotLogged private final XboxController controller;

  private final Indexer indexer;

  public RobotContainer() {

    controller = new XboxController(0);

    indexer = new Indexer();

    configureBindings();
  }

  private void configureBindings() {
    Trigger test = new Trigger(controller::getAButton);

    test.onTrue(indexer.setSpeedCommand(RPM.of(1000)));
    test.onFalse(indexer.setSpeedCommand(RPM.of(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
