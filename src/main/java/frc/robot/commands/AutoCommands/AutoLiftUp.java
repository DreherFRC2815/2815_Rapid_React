package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerLifter;

public class AutoLiftUp extends CommandBase {
    IndexerLifter indexerLifter;
    boolean finished;
    
    public AutoLiftUp(IndexerLifter i) {
        indexerLifter = i;

        addRequirements(indexerLifter);
    }

    @Override
    public void initialize() {
        indexerLifter.liftUp();
    }

    @Override
    public void execute() {
        finished = indexerLifter.checkUpperLimit();
    }
    
    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}