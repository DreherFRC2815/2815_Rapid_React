package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    CANSparkMax indexer;
    CANSparkMax lifterLeader;
    CANSparkMax lifterfollower; // possibly unnecessary
    
    RelativeEncoder indexerEncoder;
    RelativeEncoder lifterLeaderEncoder;
    RelativeEncoder lifterFollowerEncoder;

    public Indexer() {
        indexer = new CANSparkMax(5, MotorType.kBrushless);
        lifterLeader = new CANSparkMax(6, MotorType.kBrushless);
        lifterfollower = new CANSparkMax(7, MotorType.kBrushless);

        indexer.setIdleMode(IdleMode.kBrake);
        lifterLeader.setIdleMode(IdleMode.kBrake);
        lifterfollower.setIdleMode(IdleMode.kBrake);

        indexerEncoder = indexer.getEncoder();
        lifterLeaderEncoder = lifterLeader.getEncoder();
        lifterFollowerEncoder = lifterfollower.getEncoder();

        lifterfollower.follow(lifterLeader);
    }

    public void intake() {
        indexer.set(1);
    }
    
    public void off() {
        indexer.set(0);
    }

    public void outtake() {
        indexer.set(-1);
    }

    public void liftUp() {
        lifterLeader.set(1);
    }

    public void liftDown() {
        lifterLeader.set(-1);
    }
}