package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    CANSparkMax leftLeader = new CANSparkMax(1, MotorType.kBrushless);
    CANSparkMax rightLeader = new CANSparkMax(3, MotorType.kBrushless);
    
    CANSparkMax leftFollower = new CANSparkMax(2, MotorType.kBrushless);
    CANSparkMax rightFollower = new CANSparkMax(4, MotorType.kBrushless);

    RelativeEncoder leftEncoder = leftLeader.getEncoder();
    RelativeEncoder rightEncoder = rightLeader.getEncoder();

    SparkMaxPIDController leftController = leftLeader.getPIDController();
    SparkMaxPIDController rightController = rightLeader.getPIDController();

    DifferentialDrive drive = new DifferentialDrive(leftLeader, rightLeader);

    ADXRS450_Gyro gyro = new ADXRS450_Gyro();
    
    double setpoint;
    double leftEncoderPosition;
    double rightEncoderPosition;

    public DriveTrain() {
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        gyro.reset();
        gyro.calibrate();
        leftController.setP(Constants.kP);
        leftController.setI(Constants.kI);
        leftController.setD(Constants.kD);
        rightController.setP(Constants.kP);
        rightController.setI(Constants.kI);
        rightController.setD(Constants.kD);
        leftController.setOutputRange(-1, 1);
        rightController.setOutputRange(-1, 1);
    }

    public void drive(double f, double t) {
        drive.arcadeDrive(f, t);
    }

    public void drivePID(double s) {
        setpoint = s;
        leftController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        rightController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        drive.feed();
    }

    public void angleCorrect(double angle) {
        if (gyro.getAngle() < angle) {
            drive.arcadeDrive(0, 0.5);
        } else if (gyro.getAngle() > angle) {
            drive.arcadeDrive(0, -0.5);
        }
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void calibrateGyro() {
        gyro.calibrate();
    }

    public boolean leftAtSetpoint() {
        if ((leftEncoderPosition == Math.abs(setpoint + 2)) || (leftEncoderPosition == Math.abs(setpoint - 2))) {
            return true;
        } else {
            return false;
        }
    }

    public boolean rightAtSetpoint() {
        if ((rightEncoderPosition == setpoint + 2) || (rightEncoderPosition == setpoint - 2)) {
            return true;
        } else {
            return false;
        }
    }
    public double getleftVelocity() {
        return leftEncoder.getVelocity();
    }
    public double getRightVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getVelocity() {
        return (getleftVelocity() + getRightVelocity()) / 2;
    }

    public boolean atSetpoint() {
        if (leftAtSetpoint() && rightAtSetpoint()) {
            return true;
        } else {
            return false;
        }
    }

    public void update() {
        leftEncoderPosition = leftEncoder.getPosition();
        rightEncoderPosition = rightEncoder.getPosition();
    }
}