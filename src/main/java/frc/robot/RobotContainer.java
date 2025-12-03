// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.ScoringCommands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

@Logged
public class RobotContainer {

  @NotLogged private final XboxController controller;
  private final Shooter shooter;

  private final Swerve swerve;

  private boolean isBlue;

  private final SendableChooser<Command> autoChooser;

  private final Indexer indexer;

  public RobotContainer(boolean isReal) {
    controller = new XboxController(0);

    swerve = new Swerve(isReal);
    shooter = new Shooter();
    indexer = new Indexer();

    autoChooser = new SendableChooser<>();
    setupAutos();

    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.driveFieldRelativeCommand(
            controller::getLeftY, controller::getLeftX, controller::getRightX));

    indexer.setDefaultCommand(ScoringCommands.intake(indexer));

    Trigger reverseIndexer = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);
    reverseIndexer.onTrue(ScoringCommands.reverseCommand(indexer, shooter));
    reverseIndexer.onFalse(ScoringCommands.stop(shooter));

    Trigger shootHigh = new Trigger(controller::getRightBumperButton);
    shootHigh.onTrue(ScoringCommands.shootHigh(indexer, shooter));
    shootHigh.onFalse(ScoringCommands.stop(shooter));

    Trigger shootLow = new Trigger(controller::getLeftBumperButton);
    shootLow.onTrue(ScoringCommands.shootLow(indexer, shooter));
    shootLow.onFalse(ScoringCommands.stop(shooter));
  }

  private void setupAutos() {
    autoChooser.setDefaultOption("Nothing", Commands.none());

    autoChooser.addOption("Leave", Autos.leave(swerve));
    autoChooser.addOption("Score High", Autos.scoreHigh(shooter, indexer));
    autoChooser.addOption("Score High + Leave", Autos.scoreHighLeave(shooter, indexer, swerve));
    autoChooser.addOption("Score Low", Autos.scoreLow(shooter, indexer));
    autoChooser.addOption("Score Low + Leave", Autos.scoreLowLeave(shooter, indexer, swerve));

    SmartDashboard.putData(autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setAlliance() {
    isBlue = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);
    swerve.setIsBlue(isBlue);
  }
}
