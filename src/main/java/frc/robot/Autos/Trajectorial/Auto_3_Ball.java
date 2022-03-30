package frc.robot.Autos.Trajectorial;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoDrive;
import frc.robot.commands.AutoCommands.AutoDump;
import frc.robot.commands.AutoCommands.AutoIndexDrive;
import frc.robot.commands.AutoCommands.AutoLiftDown;
import frc.robot.commands.AutoCommands.AutoLiftUp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IndexerLifter;

public class Auto_3_Ball extends SequentialCommandGroup {
    
    public Auto_3_Ball(DriveTrain driveTrain, Indexer indexer, IndexerLifter indexerLifter) {
        addCommands(new AutoDump(indexer, 1));
        addCommands(new ParallelCommandGroup(
            new AutoDrive(driveTrain, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\3BallAuto1.wpilib.json"),
            new AutoLiftDown(indexerLifter)
        ));
        addCommands(new AutoIndexDrive(driveTrain, indexer, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\3BallAuto2.wpilib.json"));
        addCommands(new AutoLiftUp(indexerLifter));
        addCommands(new AutoDump(indexer, 2));
    }
}
