package frc.robot.Autos.TarmacPosition1;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AngleCorrect;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public final class Position1 extends SequentialCommandGroup {

    public Position1(DriveTrain driveTrain) {
         addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(53)));
         addCommands(new AngleCorrect(driveTrain, 180));
         addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(-110)));
    }
}