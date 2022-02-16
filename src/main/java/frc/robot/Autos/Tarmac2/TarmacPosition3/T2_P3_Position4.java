package frc.robot.Autos.Tarmac2.TarmacPosition3;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class T2_P3_Position4 extends SequentialCommandGroup {
    
    public T2_P3_Position4(DriveTrain driveTrain) {
        addCommands(new AngleCorrect(driveTrain, 125));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(91)));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(-24)));
        addCommands(new AngleCorrect(driveTrain, -40));
        addCommands(new DriveDistance(driveTrain, 85));
    }
}