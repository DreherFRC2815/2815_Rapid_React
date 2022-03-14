package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainAdvanced extends SubsystemBase {
    CANSparkMax leftLeader = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax rightLeader = new CANSparkMax(3, MotorType.kBrushless);
    
    CANSparkMax leftFollower = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax rightFollower = new CANSparkMax(4, MotorType.kBrushless);

    DifferentialDrive drive = new DifferentialDrive(
        new MotorControllerGroup(leftLeader, leftFollower),
        new MotorControllerGroup(rightLeader, rightFollower)
    );

    RelativeEncoder leftEncoder = leftLeader.getEncoder();
    RelativeEncoder rightEncoder = rightLeader.getEncoder();

    Gyro gyro = new ADXRS450_Gyro();

    DifferentialDriveOdometry odometry;

    public DriveTrainAdvanced() {
        rightFollower.follow(rightLeader);
        leftFollower.follow(leftLeader);

        rightLeader.setInverted(true);
        leftLeader.setInverted(false);

        resetEncoders();
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    }

    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    public void arcadeDrive(double f, double t) {
        drive.arcadeDrive(f, t);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftLeader.setVoltage(leftVolts);
        rightLeader.setVoltage(rightVolts);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition() / 2.0);
    }

    public RelativeEncoder getLeftEncoder() {
        return leftEncoder;
    }

    public RelativeEncoder getRightEncoder() {
        return rightEncoder;
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate();
    }
}