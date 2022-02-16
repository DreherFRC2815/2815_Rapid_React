package frc.robot.Autos.Tarmac2.TarmacCenter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveDistance;
import frc.robot.subsystems.DriveTrain;

public class T2_PC_NoExtra extends SequentialCommandGroup {
    
    public T2_PC_NoExtra(DriveTrain driveTrain) {
        addCommands(/* unload */);
        addCommands(new DriveDistance(driveTrain, Units.inchesToMeters(100)));
    }
}
