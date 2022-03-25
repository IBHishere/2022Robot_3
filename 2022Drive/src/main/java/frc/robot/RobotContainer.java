// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Date;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("RobotContainer");
// TODO: change code relating to the following line
  double targetAngle = 90;
  
XboxController  m_driveController = new XboxController(Constants.DRIVE_XBOX_CONTROLLER);
  XboxController  m_helperController = new XboxController(Constants.HELPER_XBOX_CONTROLLER);
  
  // The robot's subsystems and commands are defined here...
  private final ClimberSubsystem m_leftClimberSubsystem = new ClimberSubsystem("left", Constants.CLIMBER_MOTOR_CANLEFT_ID, true);
  private final ClimberSubsystem m_rightClimberSubsystem = new ClimberSubsystem("right", Constants.CLIMBER_MOTOR_CANRIGHT_ID, false);
  private final DriveTrainSubsystem m_tankDriveSubsystem = new DriveTrainSubsystem();
  private final BeltSubsystem m_beltSubsystem = new BeltSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem( this.m_beltSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(this.m_beltSubsystem);
  private final LimelightVisionSubsystem m_limelightVisionSubsystem = new LimelightVisionSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // ShootCommands m_shootCommand = new ShootCommands(
  //               this.m_shooterSubsystem);

  AutonSequentialCommands m_autonomous = new AutonSequentialCommands(
    this.m_tankDriveSubsystem
    , this.m_intakeSubsystem
    , this.m_shooterSubsystem
    , this.m_limelightVisionSubsystem
    , this.m_beltSubsystem);
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
//    this.m_shooterSubsystem.setDefaultCommand(m_shootCommand);
    
  }
  

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    table.getEntry("isXboxConnected").forceSetBoolean( m_helperController.isConnected() );
    
    new JoystickButton (this.m_driveController, Button.kRightBumper.value)
    .whenPressed( ()-> this.m_tankDriveSubsystem.setSpeed(DriveTrainSubsystem.highSpeedLimit));

    new JoystickButton (this.m_driveController, Button.kLeftBumper.value)
    .whenPressed( ()-> this.m_tankDriveSubsystem.setSpeed(DriveTrainSubsystem.lowSpeedLimit));
    

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
    new JoystickButton (m_helperController, Button.kY.value)
    .whenPressed( new InstantCommand(
      ()-> {
        table.getEntry("intake").forceSetString("Rt-Bumper/helper pressed: intakePull");
        this.m_intakeSubsystem.intakePull();
      }, this.m_intakeSubsystem, this.m_beltSubsystem )
    );
    // new JoystickButton (m_driveController, Button.kY.value)
    // .whenPressed( new InstantCommand(
    //   ()-> {
    //     this.m_limelightVisionSubsystem.turnOnLed();
    //   }
    // ));
    new JoystickButton (m_helperController, Button.kA.value)
    .whenPressed( new InstantCommand( 
      ()-> {
        table.getEntry("intake").forceSetString("Lt-Bumper/helper pressed: intakePush");
        this.m_intakeSubsystem.intakePush();
      }, this.m_intakeSubsystem, this.m_beltSubsystem)
    );

    new JoystickButton(m_helperController, Button.kB.value)
    .whenPressed( new InstantCommand( 
      ()-> {
        table.getEntry("intake").forceSetString("B-Button/drive pressed: intakeStop");
        this.m_intakeSubsystem.intakeStop();
      } , this.m_intakeSubsystem, this.m_beltSubsystem)
    );
    //End: intake controls

    //Shooter controls
    new JoystickButton(m_helperController, Button.kX.value)
    .whenPressed(
      
      new ShootSequence(this.m_shooterSubsystem, this.m_beltSubsystem)
    );

    new JoystickButton(m_driveController, Button.kX.value)
    .whenPressed(
      new FollowLimelightSequence(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem)
      
    // );

    //Start: Queuing controls
    //TODO: Refactor queue to toggle with single button
    // new JoystickButton(m_helperController, Button.kA.value)
    // .whenPressed(
    //   ()-> {
    //     table.getEntry("queuing").forceSetString("A-Button/helper pressed: startQueue");
    //     this.m_shooterSubsystem.toggleQueue();
    //   }
    // );

////// shhh its a secret comment you never saw this forget about it

    new JoystickButton(m_helperController, Button.kRightBumper.value)
    .whenPressed( 
      new InstantCommand( ()->{System.out.println("StartClimb, " + new Date().getTime());} )
      .andThen(
        new ParallelCommandGroup(
          new PIDClimbCommand(
            m_rightClimberSubsystem,                //subsystem
            //Constants.CLIMBDISTANCE,                // end pos
            new LinearSetpointTrajectory(
              m_rightClimberSubsystem.getPosition(), 
              Constants.CLIMBDISTANCE, 2000),
            .5,                                     // climb speed
            true,                                   // does it end (otherwise keep holding)
            "right")
          //  ,
          // new PIDClimbCommand( 
          //   m_leftClimberSubsystem,
          // //Constants.CLIMBDISTANCE,                // end pos
          // new ClimberTrajectorySetpoint(
          //   m_leftClimberSubsystem.getPosition(), 
          //     Constants.CLIMBDISTANCE, 2000, new Date())::getSetpoint,
          // .5,                                     // climb speed
          // true,                                   // does it end (otherwise keep holding)
          // "right")
        ))
    );
    
    new JoystickButton(m_helperController, Button.kLeftBumper.value)
    .whenPressed(
      //climber down
      new ParallelCommandGroup(
        new PIDClimbCommand(m_leftClimberSubsystem, ()->20.0 , 1, false, "left", ()->true)
         ,
        new PIDClimbCommand(m_rightClimberSubsystem, ()->20.0, 1, false, "right", ()->true)
      )

    );


    // new JoystickButton(m_helperController, Button.kY.value)
    // .whenPressed(
    //   ()-> {
    //     table.getEntry("queuing").forceSetString("A-Button/drive pressed: startQueue2");
    //     this.m_shooterSubsystem.toggleQueue2();
    //   }  
    // );

  
    //End: Queue controls
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    this.table.getEntry("autonomousStarted").setBoolean(true);

   
    // Command autoCommand =
    //     new InstantCommand(()->this.m_tankDriveSubsystem.zeroEncoders() )
    //     .andThen(()-> {
    //      System.out.println("limelight start");
    //      this.m_limelightVisionSubsystem.turnOnLed();   
    //    })
    //     .andThen(new FollowLimelightPidCommand(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem))
    //    .andThen( ()-> {
    //      System.out.println("limelight done");
    //      this.m_limelightVisionSubsystem.turnOffLed(); } )
    //     .andThen(new InstantCommand(()->this.m_tankDriveSubsystem.zeroEncoders() ) ) 
    //     .andThen(
    //        new DriveDistancePidCommand( this.m_tankDriveSubsystem, 1 )  //drive a distance
    //     );
        
        
        // .andThen(
       //  new ShootSequence(m_shooterSubsystem);
    
        //; /// drive distance of 10
  
    return m_autonomous;

  }
}
