package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    CANSparkMax indexer;
    
    RelativeEncoder indexerEncoder;

    public Indexer() {
        indexer = new CANSparkMax(5, MotorType.kBrushless);

        indexer.setIdleMode(IdleMode.kBrake);
    }

    public void intake() {
        indexer.set(-1);
    }

    public void outtakeDefault() {
        indexer.set(1);
    }

    public void outtake(double speed) {
        if (speed >= 0.5) {
            indexer.set(speed);
        } else if (speed >= 0) {
            indexer.set(speed + Constants.DEFAULTINDEXERSPEED);
        } else {
            indexer.set(speed + 1);
        }
    }
    
    public void off() {
        indexer.set(0);
    }
}