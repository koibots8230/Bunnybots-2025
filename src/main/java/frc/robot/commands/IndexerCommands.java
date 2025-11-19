package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IndexerConstants;
import frc.robot.commands.IndexerCommands;
import frc.robot.subsystems.Indexer;

public class IndexerCommands {
    
public static Command reverseCommand(
    Indexer indexer) {
return Commands.parallel(
    indexer.setVelocityCommand(
        IndexerConstants.REVERSE_INTAKING_SPEED));
    }
}
