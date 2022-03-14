package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerLifter;

public class AutoLiftDown extends CommandBase {
    IndexerLifter indexerLifter;
    boolean finished;
    
    public AutoLiftDown(IndexerLifter i) {
        indexerLifter = i;

        addRequirements(indexerLifter);
    }

    @Override
    public void initialize() {
        indexerLifter.liftDown();
    }

    @Override
    public void execute() {
        finished = indexerLifter.checkLowerLimit();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}