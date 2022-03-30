package frc.robot.Autos.Trajectorial;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoDump;
import frc.robot.commands.AutoCommands.AutoIndexDrive;
import frc.robot.commands.AutoCommands.AutoLiftDown;
import frc.robot.commands.AutoCommands.AutoLiftUp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IndexerLifter;

public class Tarmac_1_2_Ball_1 extends SequentialCommandGroup {

    public Tarmac_1_2_Ball_1(DriveTrain driveTrain, Indexer indexer, IndexerLifter indexerLifter) {
        addCommands(new AutoLiftDown(indexerLifter));
        addCommands(new AutoIndexDrive(driveTrain, indexer, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\2BallAuto1.wpilib.json"));
        addCommands(new AutoLiftUp(indexerLifter));
        addCommands(new AutoDump(indexer, 2));
    }
}