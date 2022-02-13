package frc.robot.Autos.TarmacPosition1;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class Position1_2 extends SequentialCommandGroup {
    
    public Position1_2(DriveTrain driveTrain) {
        addCommands(new AngleCorrect(driveTrain, 80));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(24)));
        addCommands(new AngleCorrect(driveTrain, 90));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(36)));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(-36)));
        addCommands(new AngleCorrect(driveTrain, -90));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(-24)));
        addCommands(new AngleCorrect(driveTrain, -80));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(65)));
        addCommands(new AngleCorrect(driveTrain, 92));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(120)));
        addCommands(new AngleCorrect(driveTrain, 115));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(120)));
        addCommands(new AngleCorrect(driveTrain, -45));
    }
}