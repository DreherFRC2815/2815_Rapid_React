package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerLifter extends SubsystemBase {
    CANSparkMax lifter;
    
    RelativeEncoder lifterencoder;

    DigitalInput bottomLimit;
    DigitalInput topLimit;

    public IndexerLifter() {
        lifter = new CANSparkMax(6, MotorType.kBrushless);

        lifter.setIdleMode(IdleMode.kBrake);

        bottomLimit = new DigitalInput(0);
        topLimit = new DigitalInput(1);
    }

    public void off() {
        lifter.set(0);
    }

    public void liftUp() {
        lifter.set(-1);
    }

    public void liftDown() {
        lifter.set(1);
    }

    public boolean checkUpperLimit() {
        if (topLimit.get()) {
            return true;
        }
        return false;
    }

    public boolean checkLowerLimit() {
        if (bottomLimit.get()) {
            return true;
        }
        return false;
    }

    public boolean checkLimits() {
        if (checkLowerLimit() || checkUpperLimit()) {
            return true;
        }
        return false;
    }
}