// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Autos.Tarmac1.TarmacCenter.T1_PC_Position1;
import frc.robot.Autos.Tarmac1.TarmacCenter.T1_PC_Position1_2;
import frc.robot.Autos.Tarmac1.TarmacCenter.T1_PC_Position2;
import frc.robot.Autos.Tarmac1.TarmacPosition1.T1_P1_Position1_2;
import frc.robot.Autos.Tarmac1.TarmacPosition2.T1_P2_Position2;
import frc.robot.Autos.Tarmac1.TarmacPosition2.T1_P2_Position2_1;
import frc.robot.Autos.Tarmac2.TarmacCenter.T2_PC_NoExtra;
import frc.robot.Autos.Tarmac2.TarmacPosition3.T2_P3_Position3;
import frc.robot.Autos.Tarmac2.TarmacPosition3.T2_P3_Position4;
import frc.robot.Autos.Tarmac2.TarmacPosition4.T2_P4_Position4;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveTrain;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  DriveTrain driveTrain = new DriveTrain();

  Drive drive;

  SendableChooser<SequentialCommandGroup> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser = new SendableChooser<>();
    //
    autoChooser.addOption("T1_PC_Position1_2", new T1_PC_Position1_2(driveTrain));
    autoChooser.addOption("T1_PC_Position1", new T1_PC_Position1(driveTrain));
    autoChooser.addOption("T1_PC_Position2", new T1_PC_Position2(driveTrain));
    autoChooser.addOption("T1_P1_Position1_2", new T1_P1_Position1_2(driveTrain));
    autoChooser.addOption("T1_P1_Position1", new T1_PC_Position1(driveTrain));
    autoChooser.addOption("T1_P2_Position2_1", new T1_P2_Position2_1(driveTrain));
    autoChooser.addOption("T1_P2_Position2", new T1_P2_Position2(driveTrain));
    autoChooser.addOption("T2_PC_NoExtra", new T2_PC_NoExtra(driveTrain));
    autoChooser.addOption("T2_P3_Position3", new T2_P3_Position3(driveTrain));
    autoChooser.addOption("T2_P3_Position4", new T2_P3_Position4(driveTrain));
    autoChooser.addOption("T2_P4_Position4", new T2_P4_Position4(driveTrain));
    autoChooser.setDefaultOption("Default(P4_NoExtra)", new T2_PC_NoExtra(driveTrain));

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

    drive = new Drive(driveTrain, () -> controller.getRightX(), () -> controller.getLeftY());

    driveTrain.setDefaultCommand(drive);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}