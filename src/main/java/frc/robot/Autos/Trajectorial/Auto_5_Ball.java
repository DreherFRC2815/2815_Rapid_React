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

public class Auto_5_Ball extends SequentialCommandGroup {
    
    public Auto_5_Ball(DriveTrain driveTrain, Indexer indexer, IndexerLifter indexerLifter) {
        addCommands(new AutoDump(indexer, 1));
        addCommands(new ParallelCommandGroup(
            new AutoLiftDown(indexerLifter),
            new AutoDrive(driveTrain, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\5BallAuto1.wpilib.json")
        ));
        addCommands(new AutoIndexDrive(driveTrain, indexer, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\5BallAuto2.wpilib.json"));
        addCommands(new AutoLiftUp(indexerLifter));
        addCommands(new AutoDump(indexer, 1));
        addCommands(new ParallelCommandGroup(
            new AutoLiftDown(indexerLifter),
            new AutoDrive(driveTrain, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\5BallAuto3.wpilib.json")
        ));
        addCommands(new AutoIndexDrive(driveTrain, indexer, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\5BallAuto4.wpilib.json"));
        addCommands(new AutoDrive(driveTrain, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\5BallAuto5.wpilib.json"));
        addCommands(new ParallelCommandGroup(
            new AutoLiftUp(indexerLifter),
            new AutoDrive(driveTrain, "src\\main\\java\\frc\robot\\Autos\\Trajectorial\\Routes\\output\\5BallAuto6.wpilib.json")
        ));
        addCommands(new AutoDump(indexer, 2));
    }
}
