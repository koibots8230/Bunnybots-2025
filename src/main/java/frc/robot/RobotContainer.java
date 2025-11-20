// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.IndexerCommands;
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

    Trigger reverseIndexer = new Trigger(() -> controller.getLeftTriggerAxis() > 0.15);

    reverseIndexer.onTrue(IndexerCommands.reverseCommand(indexer));

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

  private void setupAutos() {
    autoChooser.setDefaultOption("Nothing", Commands.none());

    autoChooser.addOption(
        "Test", swerve.autoDriveCommand(Meters.of(2), MetersPerSecond.of(2), Rotation2d.kZero));

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
