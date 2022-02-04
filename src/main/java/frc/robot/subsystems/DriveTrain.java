package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

    CANSparkMax leftLeader;
    CANSparkMax rightLeader;
    
    CANSparkMax leftFollower;
    CANSparkMax rightFollower;

    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;

    SparkMaxPIDController leftController;
    SparkMaxPIDController rightController;

    DifferentialDrive drive;

    ADXRS450_Gyro gyro;
    
    double setpoint;
    double leftEncoderPosition;
    double rightEncoderPosition;

    public DriveTrain() {
        leftLeader = new CANSparkMax(1, MotorType.kBrushless);
        rightLeader = new CANSparkMax(3, MotorType.kBrushless);

        leftFollower = new CANSparkMax(2, MotorType.kBrushless);
        rightFollower = new CANSparkMax(4, MotorType.kBrushless);

        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();

        leftController = leftLeader.getPIDController();
        rightController = rightLeader.getPIDController();

        drive = new DifferentialDrive(leftLeader, rightLeader);

        gyro = new ADXRS450_Gyro();

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

        leftController.setOutputRange(-0.25, 0.25);
        rightController.setOutputRange(-0.25, 0.25);

        drive.setSafetyEnabled(false);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void drive(double f, double t) {
        drive.arcadeDrive(f, t);
        SmartDashboard.putNumber("leftEncoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("rightEncoder", rightEncoder.getPosition());
    }

    public void drivePID(double s) {
        setpoint = s * (Constants.GEAR_BOX_RATIO / Constants.WHEEL_CIRCUMFERENCE) * Constants.ELLIOT_COEFFICIENT;  // MAKE SURE TO CONVERT FROM METERS TO ROTATIONS BEFORE MULTIPLYING BY THIS COEFFICIENT 1 ROTATION = THE COEFFICIENT
        leftController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        rightController.setReference(-setpoint, CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber("leftEncoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("rightEncoder", rightEncoder.getPosition());
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public boolean angleCorrect(double angle) {
        if (gyro.getAngle() < angle) {
            while (gyro.getAngle() != angle) {
                drive.arcadeDrive(0, 0.5);
            }
            return true;
        } else if (gyro.getAngle() > angle) {
            while (gyro.getAngle() != angle) {
                drive.arcadeDrive(0, -0.5);
            }
            return true;
        }
        return true;
    }

    public void resetGyro() {
        gyro.reset();
        gyro.calibrate();
    }

    public boolean leftAtSetpoint() {
        if ((leftEncoderPosition == Math.abs(setpoint + 2)) || (leftEncoderPosition == Math.abs(setpoint - 2))) {
            return true;
        }
        return false;
    }

    public boolean rightAtSetpoint() {
        if ((rightEncoderPosition == Math.abs(setpoint + 2)) || (rightEncoderPosition == Math.abs(setpoint - 2))) {
            return true;
        }
        return false;
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

    public boolean atSetpointA() {
        double left = leftEncoder.getPosition();
        double right = rightEncoder.getPosition();

        if (left == setpoint && right == setpoint) {
            if (Math.abs(left-right) <= 0.1) {
                return true;
            }
        }
        return false;
    }

    public boolean atSetpointB() {
        double left = Math.abs(leftEncoder.getPosition());
        double right = Math.abs(rightEncoder.getPosition());

        if (((left == Math.abs(setpoint + 2)) || (left == Math.abs(setpoint - 2))) &&
            ((right == Math.abs(setpoint + 2)) || (right == Math.abs(setpoint - 2)))) {

                return true;
        }
        return false;
    }

    public void update() {
        leftEncoderPosition = leftEncoder.getPosition();
        rightEncoderPosition = rightEncoder.getPosition();
    }
}