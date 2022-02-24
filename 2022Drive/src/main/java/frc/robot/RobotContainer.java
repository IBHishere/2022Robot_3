// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.AutoCommand;
// import frc.robot.commands.PIDTurnRobotCommand;
import frc.robot.commands.ShootCommands;
import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
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
// TODO: change code relating to the following line
  double targetAngle = 90;
  
XboxController  m_driveController = new XboxController(Constants.DRIVE_XBOX_CONTROLLER);
  XboxController  m_helperController = new XboxController(Constants.HELPER_XBOX_CONTROLLER);
  
  // The robot's subsystems and commands are defined here...

  private final DriveTrainSubsystem m_tankDriveSubsystem = new DriveTrainSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
   private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
 AutoCommand m_autoCommand = new AutoCommand( this.m_tankDriveSubsystem, this.m_shooterSubsystem, this.m_intakeSubsystem);
  ShootCommands m_shootCommand = new ShootCommands(
                this.m_shooterSubsystem);
 // PIDTurnRobotCommand m_PIDTurnRobotCommand = new PIDTurnRobotCommand(this.m_tankDriveSubsystem, targetAngle);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    TankDriveCommand command = new TankDriveCommand(
        this.m_tankDriveSubsystem, 
        m_driveController::getLeftY, 
        m_driveController::getRightY);
    this.m_tankDriveSubsystem.setDefaultCommand(command);
    this.m_shooterSubsystem.setDefaultCommand(m_shootCommand);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    table.getEntry("isXboxConnected").forceSetBoolean( m_helperController.isConnected() );
    
    //TODO: decide if we are doing anything with vision
    /*
    new JoystickButton (m_helperController, Button.kA.value)
    .whenPressed( 
      ()-> {
        table.getEntry("test").forceSetString("A-button pressed");
        this.m_visionSubsystem.ToggleCameraState();
      }
    );

    new JoystickButton(m_helperController, Button.kB.value)
    .whenPressed(new InstantCommand(this.m_visionSubsystem::ToggleCameraState));

    */


    //Start: Intake controls
    new JoystickButton (m_helperController, Button.kRightBumper.value)
    .whenPressed( 
      ()-> {
        table.getEntry("intake").forceSetString("Rt-Bumper/helper pressed: intakePull");
        this.m_intakeSubsystem.intakePull();
      }
    );

    new JoystickButton (m_helperController, Button.kLeftBumper.value)
    .whenPressed( 
      ()-> {
        table.getEntry("intake").forceSetString("Lt-Bumper/helper pressed: intakePush");
        this.m_intakeSubsystem.intakePush();
      }
    );

    new JoystickButton(m_helperController, Button.kB.value)
    .whenPressed(
      ()-> {
        table.getEntry("intake").forceSetString("B-Button/drive pressed: intakeStop");
        this.m_intakeSubsystem.intakeStop();
      }      
    );
    //End: intake controls

    
    //Shooter controls
    new JoystickButton(m_helperController, Button.kX.value)
    .whenPressed(
      ()-> {
        table.getEntry("shooter").forceSetString("X-Button/helper pressed: toggleShooter");
        this.m_shooterSubsystem.toggleShooter();  
      }
    );

    //Start: Queuing controls
    //TODO: Refactor queue to toggle with single button
    new JoystickButton(m_helperController, Button.kA.value)
    .whenPressed(
      ()-> {
        table.getEntry("queuing").forceSetString("A-Button/helper pressed: startQueue");
        this.m_shooterSubsystem.toggleQueue();
      }
    );

    new JoystickButton(m_helperController, Button.kY.value)
    .whenPressed(
      ()-> {
        table.getEntry("queuing").forceSetString("A-Button/drive pressed: startQueue2");
        this.m_shooterSubsystem.toggleQueue2();
      }  
    );

  
    //End: Queue controls
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return this.m_autoCommand;
  }
}
