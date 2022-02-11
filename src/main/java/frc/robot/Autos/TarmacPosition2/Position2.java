package frc.robot.Autos.TarmacPosition2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class Position2 extends SequentialCommandGroup {

    public Position2(DriveTrain driveTrain) {
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(50)));
        addCommands(new AngleCorrect(driveTrain, -113.5));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(87)));
        addCommands(new AngleCorrect(driveTrain, -120));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(90)));
    }
}