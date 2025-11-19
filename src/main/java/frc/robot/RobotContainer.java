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
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;

@Logged
public class RobotContainer {

  @NotLogged private final XboxController controller;
  private final Shooter shooter;

  private final Indexer indexer;

  public RobotContainer() {
    shooter = new Shooter();
    controller = new XboxController(0);

    indexer = new Indexer();

    configureBindings();
  }

  private void configureBindings() {

    indexer.setDefaultCommand(
        Commands.either(
                indexer.setSpeedCommand(RPM.of(0)),
                indexer.setSpeedCommand(IndexerConstants.INTAKING_SPEED),
                indexer::seePiece)
            .repeatedly());
    Trigger test = new Trigger(() -> controller.getAButton());
    test.onTrue(shooter.shootWithRPMOf(RPM.of(1000)));
    test.onFalse(shooter.shootWithRPMOf(RPM.of(0)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
