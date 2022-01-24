package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ta = table.getEntry("ta");

    private boolean target;
    private double xOffset;
    private double yOffset;
    private double area;

    private double distanceToTarget;

    private int camMode;
    private int pipeline;


    public LimeLight() {
        target = false;
        xOffset = 0.0;
        yOffset = 0.0;
        area = 0.0;

        distanceToTarget = 0.0;

        camMode = 0;
        pipeline = 0;
    }

    public double getX() {
        xOffset = tx.getDouble(0.0);
        return xOffset;
    }

    public double getY() {
        yOffset = ty.getDouble(0.0);
        return yOffset;
    }

    public boolean getTarget() {
        target = tv.getBoolean(false);
        return target;
    }

    public double getArea() {
        area = ta.getDouble(0.0);
        return area;
    }
    
    public void setPipeline(int x) {
        pipeline = x;
    }

    public int getPipeline() {
        return pipeline;
    }

    public void toggleLED(int ledMode) {
        if (ledMode == 3) {
            table.getInstance().getTable("limelight").getEntry("ledMode").setNumber(3);
        } else if (ledMode == 0) {
            table.getInstance().getTable("limelight").getEntry("ledMode").setNumber(0);
        } else {
            table.getInstance().getTable("limelight").getEntry("ledMode").setNumber(1);
        }
    }

    public void toggleCamMode() {
        if (camMode == 1) {
            table.getInstance().getTable("limelight").getEntry("camMode").setNumber(1);
        } else {
            table.getInstance().getTable("limelight").getEntry("camMode").setNumber(0);
        }
    }

    public double getDistanceToTarget() {
        distanceToTarget = (Constants.targetHeight - Constants.limelightHeight) / Math.tan(getY() + Constants.limelightAngleFromFloorParallel);
        return distanceToTarget;
    }
}