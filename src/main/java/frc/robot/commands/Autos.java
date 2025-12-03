package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

public class Autos {

  public static Command leave(Swerve swerve) {
    return swerve.autoDriveCommand(
        AutoConstants.LEAVE_DISTANCE, AutoConstants.LEAVE_VELOCITY, AutoConstants.LEAVE_HEADING);
  }

  public static Command scoreHigh(Shooter shooter, Indexer indexer) {
    return Commands.sequence(
        Commands.race(
            ScoringCommands.shootHigh(indexer, shooter),
            Commands.waitTime(AutoConstants.SHOOTING_TIME)),
        indexer.setSpeedCommand(RPM.of(0)),
        ScoringCommands.stop(shooter));
  }

  public static Command scoreHighLeave(Shooter shooter, Indexer indexer, Swerve swerve) {
    return Commands.sequence(
        Commands.race(
            ScoringCommands.shootHigh(indexer, shooter),
            Commands.waitTime(AutoConstants.SHOOTING_TIME)),
        indexer.setSpeedCommand(RPM.of(0)),
        ScoringCommands.stop(shooter),
        Autos.leave(swerve));
  }

  public static Command scoreLow(Shooter shooter, Indexer indexer) {
    return Commands.sequence(
        Commands.race(
            ScoringCommands.shootLow(indexer, shooter),
            Commands.waitTime(AutoConstants.SHOOTING_TIME)),
        indexer.setSpeedCommand(RPM.of(0)),
        ScoringCommands.stop(shooter));
  }

  public static Command scoreLowLeave(Shooter shooter, Indexer indexer, Swerve swerve) {
    return Commands.sequence(
        Commands.race(
            ScoringCommands.shootLow(indexer, shooter),
            Commands.waitTime(AutoConstants.SHOOTING_TIME)),
        indexer.setSpeedCommand(RPM.of(0)),
        ScoringCommands.stop(shooter),
        Autos.leave(swerve));
  }
}
