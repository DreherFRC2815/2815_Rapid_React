package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerLifter;

public class IndexerLift extends CommandBase {

    public IndexerLifter indexerLifter;
    public BooleanSupplier joy3;
    public BooleanSupplier joy4;
    public BooleanSupplier joy5;
    public boolean finished;

    public IndexerLift(IndexerLifter i, BooleanSupplier j3, BooleanSupplier j4, BooleanSupplier j5) {
        indexerLifter = i;
        joy3 = j3;
        joy4 = j4;
        joy5 = j5;

        addRequirements(indexerLifter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (joy3.getAsBoolean() && !indexerLifter.checkLowerLimit()) {
            indexerLifter.liftUp();
        } else if (joy4.getAsBoolean() && !indexerLifter.checkUpperLimit()) {
            indexerLifter.liftDown();
        } else if (joy5.getAsBoolean()) {
            indexerLifter.liftUp();
        } else {
            indexerLifter.off();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
} 