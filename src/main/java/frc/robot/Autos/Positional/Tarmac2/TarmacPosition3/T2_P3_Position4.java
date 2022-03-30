package frc.robot.Autos.Positional.Tarmac2.TarmacPosition3;

import edu.wpi.first.math.util.Units;
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

public class T2_P3_Position4 extends SequentialCommandGroup {
    
    public T2_P3_Position4(DriveTrain driveTrain, Indexer indexer, IndexerLifter indexerLifter) {
        addCommands(new AutoLiftDown(indexerLifter));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(78)));
        addCommands(new AutoIndex(indexer, 1));
        addCommands(new AngleCorrect(driveTrain, 140));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(85)));
        addCommands(new AutoLiftUp(indexerLifter));
        addCommands(new AutoDump(indexer, 0.5));
    }
}