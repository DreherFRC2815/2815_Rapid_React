package frc.robot.Autos.Tarmac1.TarmacPosition1;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class T1_P1_Position1 extends SequentialCommandGroup {

    public T1_P1_Position1(DriveTrain driveTrain) {
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(53)));
        addCommands(new AngleCorrect(driveTrain, 90));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(40)));
        addCommands(new AngleCorrect(driveTrain, 90));
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(70)));
    }
}