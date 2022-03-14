// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // robot constants
    public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(6) * Math.PI;
    public static final double GEAR_BOX_RATIO = 10.71;
    public static final double DEFAULTINDEXERSPEED = 0.5;

    // Encoder position at 1 revloution or 0.4788 meters
    public static final double ELLIOT_COEFFICIENT = 5/4;
    public static final double TICKS_PER_ROTATION = 360/40;

    // controller Gains
    public static final double kS = 0.20407;
    public static final double kV = 2.8133;
    public static final double kA = 0.44351;

    public static final double kP_P = 1.8777;
    public static final double kI_P = 0;
    public static final double kD_P = 0.14398;

    public static final double kP_V = 0;
    public static final double kI_V = 0;
    public static final double kD_V = 0;

    public static final double r_kP = 0.0125;
    public static final double r_kI = 0;
    public static final double r_kD = 0;

    // limelight constants (inches, degrees)
    public static final double targetHeight = 0;
    public static final double limelightHeight = 0;
    public static final double limelightAngleFromFloorParallel = 0;
    public static final double limelightAngleFromFloorPerpendicular = 0;

    // trajectory constants
    public static final double TRACK_WIDTH = 0;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
        new DifferentialDriveKinematics(TRACK_WIDTH);
    public static final double MAX_SPEED = 3;
    public static final double MAX_ACCELERATION = 3;

    public static final double ramseteB = 2;
    public static final double ramseteZeta = 0.7;
}
