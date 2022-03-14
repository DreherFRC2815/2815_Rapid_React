package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class Index extends CommandBase {

    public Indexer indexer;
    public BooleanSupplier joy1;
    public BooleanSupplier joy2;
    public boolean finished;
    public DoubleSupplier slider;

    public Index(Indexer i, BooleanSupplier j1, BooleanSupplier j2) {
        indexer = i;
        joy1 = j1;
        joy2 = j2;

        addRequirements(indexer);
    }

    public Index(Indexer i, BooleanSupplier j1, BooleanSupplier j2, DoubleSupplier jslider) {
        indexer = i;
        joy1 = j1;
        joy2 = j2;
        slider = jslider;

        addRequirements(indexer);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (joy2.getAsBoolean()) {
            indexer.intake();
        } else if (joy1.getAsBoolean()) {
            indexer.outtakeDefault();
        } else {
            indexer.off();
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
