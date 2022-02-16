package frc.robot.Autos.Tarmac2.TarmacPosition3;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class T2_P3_Position3 extends SequentialCommandGroup {
    
    public T2_P3_Position3(DriveTrain driveTrain) {
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(75)));
        addCommands(new AngleCorrect(driveTrain, 180));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(105)));
        addCommands(new AngleCorrect(driveTrain, 105));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(24)));
    }
}