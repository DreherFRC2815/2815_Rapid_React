package frc.robot.commands.AutoCommands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutoDrive extends CommandBase {
    DriveTrain driveTrain;
    RamseteCommand ramseteCommand;
    Path trajectoryJSON;
    Trajectory trajectory = new Trajectory();
    boolean finished;

    public AutoDrive(DriveTrain d, String t) {
        driveTrain = d;
        trajectoryJSON = Filesystem.getDeployDirectory().toPath().resolve(t);
        try {
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryJSON);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, e.getStackTrace());
        }

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        ramseteCommand =
            new RamseteCommand(
            trajectory,
            driveTrain::getPose,
            new RamseteController(Constants.ramseteB, Constants.ramseteZeta),
            new SimpleMotorFeedforward(
                Constants.kS,
                Constants.kV,
                Constants.kA
            ),
            Constants.DRIVE_KINEMATICS,
            driveTrain::getWheelSpeeds,
            new PIDController(Constants.kP_V, Constants.kI_V, Constants.kD_V),
            new PIDController(Constants.kP_V, Constants.kI_V, Constants.kD_V),
            driveTrain::tankDriveVolts,
            driveTrain
        );

        ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
        ramseteCommand.execute();
    }

    @Override
    public void execute() {
        if (ramseteCommand.isFinished()) {
            finished = true;
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}