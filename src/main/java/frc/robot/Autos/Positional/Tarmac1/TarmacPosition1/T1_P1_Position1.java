package frc.robot.Autos.Positional.Tarmac1.TarmacPosition1;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AngleCorrect;
import frc.robot.commands.AutoCommands.AutoDump;
import frc.robot.commands.AutoCommands.AutoIndex;
import frc.robot.commands.AutoCommands.AutoLiftDown;
import frc.robot.commands.AutoCommands.AutoLiftUp;
import frc.robot.commands.AutoCommands.DriveDistance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IndexerLifter;

public class T1_P1_Position1 extends SequentialCommandGroup {

    public T1_P1_Position1(DriveTrain driveTrain, Indexer indexer, IndexerLifter indexerLifter) {
        addCommands(new ParallelCommandGroup(
            new DriveDistance(driveTrain, Units.inchesToMeters(53)),
            new AutoLiftDown(indexerLifter),
            new AutoIndex(indexer, 3)
        ));
        addCommands(new AngleCorrect(driveTrain, 90));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(40)));
        addCommands(new AngleCorrect(driveTrain, 90));
        addCommands(new ParallelCommandGroup(
            new DriveDistance(driveTrain, Units.inchesToMeters(70)),
            new AutoLiftUp(indexerLifter)
        ));
        addCommands(new AutoDump(indexer, 0.5));
    }
}