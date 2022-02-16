package frc.robot.Autos.Tarmac1.TarmacCenter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class T1_PC_Position2 extends SequentialCommandGroup {

    public T1_PC_Position2(DriveTrain driveTrain) {
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(5)));
        addCommands(new AngleCorrect(driveTrain, 45));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(95)));
        addCommands(new AngleCorrect(driveTrain, 180));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(95)));
        addCommands(new AngleCorrect(driveTrain, -45));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(5)));
    }
}
