package frc.robot.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class Test extends SequentialCommandGroup{

    public Test(DriveTrain driveTrain) {
        addCommands(new DriveDistance(driveTrain, 1));
    }
}