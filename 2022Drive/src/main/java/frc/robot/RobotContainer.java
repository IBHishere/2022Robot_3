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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methFods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public enum Mode {
    Undefined,
    TeleOp,
    Test,
    Autonomous,
    Practice
  }

  
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("RobotContainer");
  
  
XboxController  m_driveController = new XboxController(Constants.DRIVE_XBOX_CONTROLLER);
  XboxController  m_helperController = new XboxController(Constants.HELPER_XBOX_CONTROLLER);
  
  // The robot's subsystems and commands are defined here...
  private final limelight test = new limelight();
  // private final ClimberSubsystem m_leftClimberSubsystem = new ClimberSubsystem("left", Constants.CLIMBER_MOTOR_CANLEFT_ID, true);
  // private final ClimberSubsystem m_rightClimberSubsystem = new ClimberSubsystem("right", Constants.CLIMBER_MOTOR_CANRIGHT_ID, false);
  // private final DriveTrainSubsystem m_tankDriveSubsystem = new DriveTrainSubsystem();
  // private final BeltSubsystem m_beltSubsystem = new BeltSubsystem();
  // private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem( this.m_beltSubsystem);
  // private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(this.m_beltSubsystem);
  // private final LimelightVisionSubsystem m_limelightVisionSubsystem = new LimelightVisionSubsystem();
  // //private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  // private final QueueFeederWheelSubsystem m_queueFeederWheelSubsystem = new QueueFeederWheelSubsystem();
  // ShootCommands m_shootCommand = new ShootCommands(
  //               this.m_shooterSubsystem);

  // AutonSequentialCommands m_autonomous = new AutonSequentialCommands(
  //   this.m_tankDriveSubsystem
  //   , this.m_intakeSubsystem
  //   , this.m_shooterSubsystem
  //   , this.m_limelightVisionSubsystem
  //   , this.m_beltSubsystem
  //   , this.m_queueFeederWheelSubsystem
  //   );
  private edu.wpi.first.wpilibj2.command.button.Button whenPressed;
 // PIDTurnRobotCommand m_PIDTurnRobotCommand = new PIDTurnRobotCommand(this.m_tankDriveSubsystem, targetAngle);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  public RobotContainer() {
    // Configure the button bindings
    
    
    configureButtonBindings();
    
    // TankDriveCommand command = new TankDriveCommand(
    //     this.m_tankDriveSubsystem, 
    //     m_driveController::getLeftY, 
    //     m_driveController::getRightY);
    
    // this.m_tankDriveSubsystem.setDefaultCommand(command);
    // this.m_rightClimberSubsystem.setDefaultCommand(
    //   new WindClimberCommand(
    //       this.m_rightClimberSubsystem, 
    //       this.m_helperController::getRightY)
    // );

    // this.m_leftClimberSubsystem.setDefaultCommand(
    //   new WindClimberCommand(
    //       this.m_leftClimberSubsystem,  
    //       this.m_helperController::getLeftY)
    // );
 


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
    
    // new JoystickButton (this.m_driveController, Button.kRightBumper.value)
    // .whenPressed( ()-> this.m_tankDriveSubsystem.setSpeed(DriveTrainSubsystem.highSpeedLimit));

    // new JoystickButton (this.m_driveController, Button.kLeftBumper.value)
    // .whenPressed( ()-> this.m_tankDriveSubsystem.setSpeed(DriveTrainSubsystem.lowSpeedLimit));
    

    //TODO: decide if we are doing anything with vision
    
    // new JoystickButton (m_helperController, Button.kA.value)
    // .whenPressed( 
    //   ()-> {
    //     table.getEntry("test").forceSetString("A-button pressed");
    //     this.m_visionSubsystem.ToggleCameraState();
    //   }
    // );

    new JoystickButton(m_helperController, Button.kB.value)
    .whenPressed(()->{
        limelight.test();
    }
     ); 
    


    //Start: Intake controls
    // new JoystickButton (m_helperController, Button.kY.value)
    // .whenPressed( new InstantCommand(
    //   ()-> {
    //     table.getEntry("intake").forceSetString("Rt-Bumper/helper pressed: intakePull");
    //     this.m_intakeSubsystem.intakePull();
    //   }, this.m_intakeSubsystem, this.m_beltSubsystem )
    // );

    // new JoystickButton (m_helperController, Button.kA.value)
    // .whenPressed( new InstantCommand( 
    //   ()-> {
    //     table.getEntry("intake").forceSetString("Lt-Bumper/helper pressed: intakePush");
    //     this.m_intakeSubsystem.intakePush();
    //   }, this.m_intakeSubsystem, this.m_beltSubsystem)
    // );

    // new JoystickButton(m_helperController, Button.kB.value)
    // .whenPressed( new InstantCommand( 
    //   ()-> {
    //     table.getEntry("intake").forceSetString("B-Button/drive pressed: intakeStop");
    //     this.m_intakeSubsystem.intakeStop();
    //   } , this.m_intakeSubsystem, this.m_beltSubsystem)
    // );
    // //End: intake controls

    // //Shooter controls
    // new JoystickButton(m_helperController, Button.kX.value)
    // .whenPressed(
    //   //new ShooterTestCommand(this.m_shooterSubsystem)
    
    //   new ShootSequence(this.m_shooterSubsystem, this.m_beltSubsystem, this.m_queueFeederWheelSubsystem)
    // );

    // new JoystickButton(m_driveController, Button.kX.value)
    // .whenPressed(
      
    //   // new FollowLimelightSequence(this.m_tankDriveSubsystem, this.m_limelightVisionSubsystem)
    // );

    // new JoystickButton(m_helperController, Button.kRightBumper.value)
    // .whenPressed( 
    //   climberMoveCommand(Constants.CLIMBDISTANCE, 500, true, getExtendClimberPidSettings()) // fast going up
    // );

    
    // new JoystickButton(m_helperController, Button.kLeftBumper.value)
    // .whenPressed(
    //   climberMoveCommand(0, 5000, false, getLiftPidSettings()) //slow going down as robot is pulling up
    // );


    // new JoystickButton(m_helperController, Button.kRightStick.value)
    // .whenPressed(
    //   zeroClimber(this.m_rightClimberSubsystem, -20)
    // );

    // new JoystickButton(m_helperController, Button.kLeftStick.value)
    //   .whenPressed(
    //     zeroClimber(this.m_leftClimberSubsystem, -20)
    // );

    // new JoystickButton(m_helperController, Button.kStart.value)
    //   .whenPressed(()->
    //    {
    //      table.getEntry("tetsts").setString("asdasdasd");
    //       ShooterSubsystem.ToggleGoalMode();
    //       System.out.println("tring to toggle goale mode");}



      
    
    //End: Queue controls
    
  }


  
  private PidSettings getLiftPidSettings() {
    return new PidSettings(25.0/(80.0), 100.0/80.0, .3/80.0);
  }


  private PidSettings getExtendClimberPidSettings() {
    return new PidSettings(14.0/(80.0), 0, 0);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * //@return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   this.table.getEntry("autonomousStarted").setBoolean(true);
    //return m_autonomous;
    // return new AutonSequentialCommands(
    //   this.m_tankDriveSubsystem
    //   , this.m_intakeSubsystem
    //   , this.m_shooterSubsystem
    //   , this.m_limelightVisionSubsystem
    //   , this.m_beltSubsystem
    //   , this.m_queueFeederWheelSubsystem
    //   );
  

  // private SequentialCommandGroup climberMoveCommand(
  //         double positionTarget
  //       , double moveTimeInMilliseconds
  //       , boolean doesItEverEnd
  //       , PidSettings pidSettings) {
  //   return new InstantCommand( ()->{System.out.println("StartClimb, " + new Date().getTime());} )
  //   .andThen(
  //     new ParallelCommandGroup(

  //       new PIDClimbCommand(
  //         m_rightClimberSubsystem,                //subsystem
  //         new LinearSetpointTrajectory(
  //           m_rightClimberSubsystem.getPosition(), 
  //           positionTarget, moveTimeInMilliseconds, "rightClimber"),
  //         1,                                     // climb speed
  //         doesItEverEnd,                                   // does it end (otherwise keep holding)
  //         "right",
  //         pidSettings)
  //        ,
  //       new PIDClimbCommand( 
  //         m_leftClimberSubsystem,
  //         new LinearSetpointTrajectory(
  //           m_leftClimberSubsystem.getPosition(), 
  //           positionTarget, moveTimeInMilliseconds, "leftClimber"),
  //       1,                                     // climb speed
  //       doesItEverEnd,                                   // does it end (otherwise keep holding)
  //       "left",
  //       pidSettings)
  //     ));
  // }

  // private Command zeroClimber(ClimberSubsystem climberSubsystem, double windAmount) {
  //   return 
  //    new InstantCommand( 
  //      ()-> {
  //        climberSubsystem.zeroEncoder();
  //        //System.out.println("Zero");
  //       } )
  //    ;
   
  // }
}
