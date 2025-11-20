package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;

public class ScoringCommands {

  public static Command reverseCommand(Indexer indexer, Shooter shooter) {
    return Commands.parallel(
        indexer.setSpeedCommand(IndexerConstants.REVERSE_INTAKING_SPEED).repeatedly(),
        shooter.setVelocityCommand(ShooterConstants.REVERSE_SPEED));
  }

  public static Command intake(Indexer indexer) {
    return Commands.either(
            indexer.setSpeedCommand(RPM.of(0)),
            indexer.setSpeedCommand(IndexerConstants.INTAKING_SPEED),
            indexer::seePiece)
        .repeatedly();
  }

  public static Command shootHigh(Indexer indexer, Shooter shooter) {
    return Commands.sequence(
        shooter.setVelocityCommand(ShooterConstants.HIGH_GOAL_SPEED),
        Commands.waitSeconds(0.1),
        indexer.setSpeedCommand(IndexerConstants.SHOOT_SPEED).repeatedly());
  }

  public static Command shootLow(Indexer indexer, Shooter shooter) {
    return Commands.sequence(
        shooter.setVelocityCommand(ShooterConstants.LOW_GOAL_SPEED),
        Commands.waitSeconds(0.1),
        indexer.setSpeedCommand(IndexerConstants.SHOOT_SPEED).repeatedly());
  }

  public static Command stop(Shooter shooter) {
    return shooter.setVelocityCommand(RPM.of(0));
  }
}
