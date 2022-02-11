// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("datatable");

  XboxController  m_driveController = new XboxController(Constants.DRIVE_XBOX_CONTROLLER);
  XboxController  m_helperController = new XboxController(Constants.HELPER_XBOX_CONTROLLER);
  
  // The robot's subsystems and commands are defined here...
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final DriveTrainSubsystem m_tankDriveSubsystem = new DriveTrainSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final AutonomousSubsystem m_autonomousSubsystem = new AutonomousSubsystem();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    IntakeCommand m_intakecommand = new IntakeCommand(
              this.m_intakeSubsystem);
      this.m_intakeSubsystem.setDefaultCommand(m_intakecommand);
    
    TankDriveCommand command = new TankDriveCommand(
               this.m_tankDriveSubsystem, m_driveController::getLeftY, m_driveController::getRightY);
    this.m_tankDriveSubsystem.setDefaultCommand(command);
    AutonomousCommand m_autonomousCommand = new AutonomousCommand(m_autonomousSubsystem);
    this.m_autonomousSubsystem.setDefaultCommand(m_autonomousCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    table.getEntry("isXboxConnected").forceSetBoolean( m_helperController.isConnected() );
    
    new JoystickButton (m_helperController, Button.kA.value)
    .whenPressed( 
      ()-> {
        table.getEntry("test").forceSetString("A-button pressed");
        this.m_visionSubsystem.ToggleCameraState();
      }
    );

    new JoystickButton (m_helperController, Button.kRightStick.value)
    .whileHeld( 
      ()-> {
        table.getEntry("test").forceSetString("Rt-button pressed");
        this.m_intakeSubsystem.intakePull();
        //do not uncomment the following line of code unless you know what you are doing
        //this.m_intakeSubsystem.startIntake();
      }
    );
    new JoystickButton (m_helperController, Button.kLeftStick.value)
    .whileHeld( 
      ()-> {
        table.getEntry("test").forceSetString("Lt-button pressed");
        this.m_intakeSubsystem.intakePush();
        
      }
    );

    new JoystickButton(m_helperController, Button.kB.value)
    .whenPressed(new InstantCommand(this.m_visionSubsystem::ToggleCameraState));

    new JoystickButton(m_helperController, Button.kX.value)
    .whenPressed(
      ()-> {
         this.m_shooterSubsystem.startShooter();
         
      }
      
    );
    new JoystickButton(m_helperController, Button.kY.value)
    .whenPressed(
      ()-> {
         this.m_shooterSubsystem.stopShooter();
         
      }
    );

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
