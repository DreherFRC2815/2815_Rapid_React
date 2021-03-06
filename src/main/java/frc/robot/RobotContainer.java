// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Autos.Test;
import frc.robot.Autos.Positional.Tarmac1.TarmacCenter.T1_PC_Position1;
import frc.robot.Autos.Positional.Tarmac1.TarmacCenter.T1_PC_Position1_2;
import frc.robot.Autos.Positional.Tarmac1.TarmacCenter.T1_PC_Position2;
import frc.robot.Autos.Positional.Tarmac1.TarmacPosition1.T1_P1_Position1;
import frc.robot.Autos.Positional.Tarmac1.TarmacPosition1.T1_P1_Position1_2;
import frc.robot.Autos.Positional.Tarmac1.TarmacPosition2.T1_P2_Position2;
import frc.robot.Autos.Positional.Tarmac1.TarmacPosition2.T1_P2_Position2_1;
import frc.robot.Autos.Positional.Tarmac2.TarmacCenter.T2_PC_NoExtra;
import frc.robot.Autos.Positional.Tarmac2.TarmacPosition3.T2_P3_Position3;
import frc.robot.Autos.Positional.Tarmac2.TarmacPosition3.T2_P3_Position4;
import frc.robot.Autos.Positional.Tarmac2.TarmacPosition4.T2_P4_Position4;
import frc.robot.Autos.Trajectorial.Auto_3_Ball;
import frc.robot.Autos.Trajectorial.Auto_4_Ball;
import frc.robot.Autos.Trajectorial.Auto_5_Ball;
import frc.robot.Autos.Trajectorial.Tarmac_1_2_Ball_1;
import frc.robot.Autos.Trajectorial.Tarmac_1_2_Ball_2;
import frc.robot.Autos.Trajectorial.Tarmac_2_2_Ball_1;
import frc.robot.commands.Climb;
import frc.robot.commands.Drive;
import frc.robot.commands.Index;
import frc.robot.commands.IndexerLift;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.IndexerLifter;
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveTrain driveTrain = new DriveTrain();
  Indexer indexer = new Indexer();
  IndexerLifter indexerLifter = new IndexerLifter();
  Climber climber = new Climber();

  Drive drive;
  Index index;
  IndexerLift indexerLift;
  Climb climb;

  SendableChooser<CommandBase> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SmartDashboard.putNumber("Rotational P", Constants.r_kP);
    SmartDashboard.putNumber("Rotational I", Constants.r_kI);
    SmartDashboard.putNumber("Rotational D", Constants.r_kD);

    autoChooser = new SendableChooser<>();
    autoChooser.addOption("T1_PC_Position1_2", new T1_PC_Position1_2(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T1_PC_Position1", new T1_PC_Position1(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T1_PC_Position2", new T1_PC_Position2(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T1_P1_Position1_2", new T1_P1_Position1_2(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T1_P1_Position1", new T1_P1_Position1(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T1_P2_Position2_1", new T1_P2_Position2_1(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T1_P2_Position2", new T1_P2_Position2(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T2_PC_NoExtra", new T2_PC_NoExtra(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T2_P3_Position3", new T2_P3_Position3(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T2_P3_Position4", new T2_P3_Position4(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("T2_P4_Position4", new T2_P4_Position4(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("Tarmac_1_2_Ball_1", new Tarmac_1_2_Ball_1(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("Tarmac_1_2_Ball_2", new Tarmac_1_2_Ball_2(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("Tarmac_2_2_Bal_1", new Tarmac_2_2_Ball_1(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("Auto_3_Ball", new Auto_3_Ball(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("Auto_4_Ball", new Auto_4_Ball(driveTrain, indexer, indexerLifter));
    autoChooser.addOption("Auto_5_Ball", new Auto_5_Ball(driveTrain, indexer, indexerLifter));
    autoChooser.setDefaultOption("Test", new Test(indexerLifter));

    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    XboxController controller = new XboxController(0);
    Joystick joystick = new Joystick(1);

    drive = new Drive(driveTrain, () -> controller.getLeftY(), () -> controller.getRightX());
    index = new Index(indexer, () -> joystick.getRawButton(1), () -> joystick.getRawButton(2));
    indexerLift = new IndexerLift(indexerLifter, () -> joystick.getRawButton(3), () -> joystick.getRawButton(4), () -> joystick.getRawButton(5));
    climb = new Climb(climber, () -> controller.getLeftBumper(), () -> controller.getRightBumper());

    driveTrain.setDefaultCommand(drive);
    indexer.setDefaultCommand(index);
    indexerLifter.setDefaultCommand(indexerLift);
    climber.setDefaultCommand(climb);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}