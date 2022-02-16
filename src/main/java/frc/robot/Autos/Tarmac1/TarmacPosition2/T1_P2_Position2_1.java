package frc.robot.Autos.Tarmac1.TarmacPosition2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class T1_P2_Position2_1 extends SequentialCommandGroup {
    
    public T1_P2_Position2_1(DriveTrain driveTrain) {
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(50)));
        addCommands(new AngleCorrect(driveTrain, 45));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(-50)));
        addCommands(new AngleCorrect(driveTrain, 180));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(50)));
        addCommands(new AngleCorrect(driveTrain, -120));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(120)));
        addCommands(new AngleCorrect(driveTrain, -120));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(120)));
        addCommands(new AngleCorrect(driveTrain, -70));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(110)));

    }
}