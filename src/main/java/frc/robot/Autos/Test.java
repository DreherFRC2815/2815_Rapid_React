package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoLiftDown;
import frc.robot.commands.AutoCommands.AutoLiftUp;
import frc.robot.subsystems.IndexerLifter;

public class Test extends SequentialCommandGroup {
    
    public Test(IndexerLifter indexerLifter) {
        addCommands(new AutoLiftUp(indexerLifter));
        addCommands(new AutoLiftDown(indexerLifter));
    }
}