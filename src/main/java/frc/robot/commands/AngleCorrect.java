package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AngleCorrect extends CommandBase {
    DriveTrain driveTrain;
    double angle;
    boolean finished;

    public AngleCorrect(DriveTrain DT, double a) {
        driveTrain = DT;
        angle = a;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetGyro();
        driveTrain.resetEncoders();
        driveTrain.setAngle(angle);
    }

    @Override
    public void execute() {
        if (!finished) {
            finished = driveTrain.rotateSlow();
        }
        driveTrain.update();
    }
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}