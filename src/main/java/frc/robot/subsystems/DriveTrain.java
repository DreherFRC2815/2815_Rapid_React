package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

    SimpleMotorFeedforward feedforward;
    PIDController gyroController;
    
    double setpoint;
    double angle;
    double leftEncoderPosition;
    double rightEncoderPosition;
    double desiredAngle;
    double angleSetpoint;

    double kP;
    double kI;
    double kD;

    DifferentialDrivetrainSim driveSim = DifferentialDrivetrainSim.createKitbotSim(
        KitbotMotor.kDoubleNEOPerSide,
        KitbotGearing.k10p71,
        KitbotWheelSize.kSixInch,
        null
    );

    Gyro gyro;

    DifferentialDriveOdometry odometry;

    RamseteCommand ramseteCommand;

    double leftPosition;
    double rightPosition;

    Field2d field = new Field2d();
    
    public DriveTrain() {
        leftLeader = new CANSparkMax(1, MotorType.kBrushless);
        rightLeader = new CANSparkMax(3, MotorType.kBrushless);

        leftFollower = new CANSparkMax(2, MotorType.kBrushless);
        rightFollower = new CANSparkMax(4, MotorType.kBrushless);

        leftLeader.setIdleMode(IdleMode.kBrake);
        rightLeader.setIdleMode(IdleMode.kBrake);

        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();

        leftController = leftLeader.getPIDController();
        rightController = rightLeader.getPIDController();

        rightLeader.setInverted(true);
        leftLeader.setInverted(false);

        drive = new DifferentialDrive(leftLeader, rightLeader);

        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
        gyroController = new PIDController(0, 0, 0);
        feedforward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
        leftController.setP(Constants.kP_P);
        leftController.setI(Constants.kI_P);
        leftController.setD(Constants.kD_P);
        rightController.setP(Constants.kP_P);
        rightController.setI(Constants.kI_P);
        rightController.setD(Constants.kD_P);

        leftController.setOutputRange(-0.4, 0.4);
        rightController.setOutputRange(-0.4, 0.4);

        drive.setSafetyEnabled(false);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void drive(double f, double t) {
        // drive.arcadeDrive(t, f);
        drive.arcadeDrive(-f, t);
        SmartDashboard.putNumber("leftEncoder", leftEncoder.getPosition());
        SmartDashboard.putNumber("rightEncoder", rightEncoder.getPosition());
    }

    public void setSetpoint(double s) {
        setpoint = s * (Constants.GEAR_BOX_RATIO / Constants.WHEEL_CIRCUMFERENCE) * Constants.ELLIOT_COEFFICIENT;  // MAKE SURE TO CONVERT FROM METERS TO ROTATIONS BEFORE MULTIPLYING BY THIS COEFFICIENT 1 ROTATION = THE COEFFICIENT
    }

    public void drivePID() {
        leftController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
        rightController.setReference(setpoint, CANSparkMax.ControlType.kPosition);
    
        SmartDashboard.putNumber("setpoint", setpoint);
        SmartDashboard.putNumber("leftEncoder", Math.abs(leftEncoder.getPosition()));
        SmartDashboard.putNumber("rightEncoder", Math.abs(rightEncoder.getPosition()));
    }

    public boolean rotatePID() {
        kP = SmartDashboard.getNumber("Rotational P", Constants.r_kP);
        kI = SmartDashboard.getNumber("Rotational I", Constants.r_kI);
        kD = SmartDashboard.getNumber("Rotational D", Constants.r_kD);
        double currentAngle = gyro.getAngle();
        double finalAngle = angle + desiredAngle;
        SmartDashboard.putNumber("final angle", finalAngle);
        SmartDashboard.putNumber("current angle", currentAngle);
        SmartDashboard.putNumber("Rotation Rate", gyro.getRate());

        gyroController.setPID(kP, kI, kD);

        if (finalAngle < 0) {
            if (currentAngle <= finalAngle) {
                drive.arcadeDrive(0, 0);
                return true;
            }
            drive.arcadeDrive(gyroController.calculate(currentAngle, finalAngle) + feedforward.calculate(Math.PI, 2), 0);
            // drive.arcadeDrive(0, -gyroController.calculate(currentAngle, finalAngle));
        }

        if (finalAngle > 0) {
            if (currentAngle >= finalAngle) {
                drive.arcadeDrive(0, 0);
                return true; 
            }
            drive.arcadeDrive(gyroController.calculate(currentAngle, finalAngle) + feedforward.calculate(Math.PI, 2), 0);
            // drive.arcadeDrive(0, gyroController.calculate(currentAngle, finalAngle));
        }
        return false;
    }

    public boolean rotate() {
        double currentAngle = gyro.getAngle();
        double finalAngle = angle + desiredAngle;
        SmartDashboard.putNumber("final angle", finalAngle);
        SmartDashboard.putNumber("current angle", gyro.getAngle());
        if (Math.abs(currentAngle) >= Math.abs(finalAngle)) {
            drive.arcadeDrive(0, 0);
            return true;
        }

        if (finalAngle < 0) {
            drive.arcadeDrive(-0.5, 0);
        }

        if (finalAngle > 0) {
            drive.arcadeDrive(0.5, 0);
        }
        return false;
    }

    public boolean rotateSlow() {
        double currentAngle = gyro.getAngle();
        double finalAngle = angle + desiredAngle;
        SmartDashboard.putNumber("final angle", finalAngle);
        SmartDashboard.putNumber("current angle", gyro.getAngle());
        if (Math.abs(currentAngle) >= Math.abs(finalAngle)) {
            drive.arcadeDrive(0, 0);
            return true;
        }

        if (finalAngle < 0) {
            drive.arcadeDrive(-0.25, 0);
        }

        if (finalAngle > 0) {
            drive.arcadeDrive(0.25, 0);
        }
        return false;
    }

    public void resetGyro() {
        gyro.reset();
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

    public boolean atSetpoint() {
        if (Math.abs(setpoint) <= Math.abs(leftEncoder.getPosition())) {
            return true;
        }
        return false;
    }

    public void update() {
        leftEncoderPosition = Math.abs(leftEncoder.getPosition());
        rightEncoderPosition = Math.abs(rightEncoder.getPosition());
        SmartDashboard.updateValues();
    }

    public void setAngle(double a) {
        desiredAngle = a;
    }

    public void setCurrentHedding() {
        angle = gyro.getAngle();
    }

    public void setRotationalPID(double kP, double kI, double kD) {
        gyroController.setPID(kP, kI, kD);
    }

    public void periodic() {
        leftPosition = leftEncoder.getPosition() * (Constants.GEAR_BOX_RATIO / Constants.WHEEL_CIRCUMFERENCE) * Constants.ELLIOT_COEFFICIENT;
        rightPosition = rightEncoder.getPosition() * (Constants.GEAR_BOX_RATIO / Constants.WHEEL_CIRCUMFERENCE) * Constants.ELLIOT_COEFFICIENT;
        odometry.update(gyro.getRotation2d(), leftPosition, rightPosition);
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