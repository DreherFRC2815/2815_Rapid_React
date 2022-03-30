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

public class Auto_4_Ball extends SequentialCommandGroup {
    
    public Auto_4_Ball(DriveTrain driveTrain, Indexer indexer, IndexerLifter indexerLifter) {
        addCommands(new AutoLiftDown(indexerLifter));
        addCommands(new AutoIndexDrive(driveTrain, indexer, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\4BallAuto1.wpilib.json"));
        addCommands(new AutoLiftUp(indexerLifter));
        addCommands(new ParallelCommandGroup(
            new AutoLiftDown(indexerLifter),
            new AutoDrive(driveTrain, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\4BallAuto2.wpilib.json")
        ));
        addCommands(new AutoIndexDrive(driveTrain, indexer, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\4BallAuto3.wpilib.json"));
        addCommands(new AutoLiftUp(indexerLifter));
        addCommands(new AutoDump(indexer, 2));
    }
}
