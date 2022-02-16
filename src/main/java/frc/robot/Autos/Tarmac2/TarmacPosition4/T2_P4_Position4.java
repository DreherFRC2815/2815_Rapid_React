package frc.robot.Autos.Tarmac2.TarmacPosition4;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class T2_P4_Position4 extends SequentialCommandGroup {
    
    public T2_P4_Position4(DriveTrain driveTrain) {
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(53)));
        addCommands(new AngleCorrect(driveTrain, 90));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(40)));
        addCommands(new AngleCorrect(driveTrain, 90));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(70)));
    }
}
