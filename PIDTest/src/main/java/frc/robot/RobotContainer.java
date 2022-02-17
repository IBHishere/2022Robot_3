// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.TestPIDCommand;
import frc.robot.subsystems.TestPIDPositionSubsystem;
import frc.robot.subsystems.TestPIDSpeedSubsystem;
import frc.robot.subsystems.TestSingleMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */



 public class RobotContainer {
 
  enum PidMode  {
    PositionPidSubsystem,
    SpeedPidSubsystem,
    PidCommand
  }
 
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("datatable");

  XboxController  m_helperController = new XboxController(0);
 
  PidMode m_currentMode = PidMode.PositionPidSubsystem;

  // The robot's subsystems and commands are defined here...
  private final TestSingleMotorSubsystem m_testSingleMotorSubsystem = new TestSingleMotorSubsystem();
   
  private final TestPIDSpeedSubsystem m_testPidSpeedSubsystem = new TestPIDSpeedSubsystem(m_testSingleMotorSubsystem);
  private final TestPIDPositionSubsystem m_testPidPositionSubsystem = new TestPIDPositionSubsystem(m_testSingleMotorSubsystem);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  
  }

  private void togglePidMode() {
    this.m_currentMode = this.m_currentMode == PidMode.SpeedPidSubsystem ? PidMode.PositionPidSubsystem :
                         (this.m_currentMode == PidMode.PositionPidSubsystem ? PidMode.PidCommand :
                         /*else*/ PidMode.SpeedPidSubsystem);

    System.out.println("Current mode:  " + this.m_currentMode.toString());
  }

  private PidMode getMode() { return this.m_currentMode;}
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    table.getEntry("isXboxConnected").forceSetBoolean( m_helperController.isConnected() );

    new JoystickButton(m_helperController, Button.kB.value)
      .whenPressed( () -> {
        //System.out.println("Test");
        this.togglePidMode();
      } );
    
    //enable
    new JoystickButton(m_helperController, Button.kA.value)
      .whenPressed(
        new SelectCommand(
          Map.ofEntries(
            Map.entry(PidMode.PositionPidSubsystem, new InstantCommand(this.m_testPidPositionSubsystem::enable, this.m_testPidPositionSubsystem, this.m_testSingleMotorSubsystem)),
            Map.entry(PidMode.SpeedPidSubsystem, new InstantCommand(this.m_testPidSpeedSubsystem::enable, this.m_testPidSpeedSubsystem, this.m_testSingleMotorSubsystem)),
            Map.entry(PidMode.PidCommand, new TestPIDCommand(this.m_testSingleMotorSubsystem) )
          ), this::getMode));
    
    new JoystickButton(m_helperController, Button.kX.value)
      .whenPressed(
        new SelectCommand(
          Map.ofEntries(
            Map.entry(PidMode.PositionPidSubsystem, new InstantCommand(this.m_testPidPositionSubsystem::disable, this.m_testPidPositionSubsystem, this.m_testSingleMotorSubsystem)),
            Map.entry(PidMode.SpeedPidSubsystem, new InstantCommand(this.m_testPidSpeedSubsystem::disable, this.m_testPidSpeedSubsystem, this.m_testSingleMotorSubsystem)),
            Map.entry(PidMode.PidCommand, new InstantCommand( ()-> {System.out.println("PidMode.PidCommand - nothing to disable"); } ))
        ), this::getMode));
        

    new JoystickButton(m_helperController, Button.kRightBumper.value)
    .whenPressed(
      new SelectCommand(
        Map.ofEntries(
          Map.entry(PidMode.PositionPidSubsystem, new InstantCommand(this.m_testPidPositionSubsystem::increaseSetpoint, this.m_testPidPositionSubsystem, this.m_testSingleMotorSubsystem)),
          Map.entry(PidMode.SpeedPidSubsystem, new InstantCommand(this.m_testPidSpeedSubsystem::increaseSetpoint, this.m_testPidSpeedSubsystem, this.m_testSingleMotorSubsystem)),
          Map.entry(PidMode.PidCommand, new InstantCommand( ()-> { TestPIDCommand.increaseSetpoint();} ))
      ), this::getMode));


    new JoystickButton(m_helperController, Button.kLeftBumper.value)
    .whenPressed(
      new SelectCommand(
        Map.ofEntries(
          Map.entry(PidMode.PositionPidSubsystem, new InstantCommand(this.m_testPidPositionSubsystem::decreaseSetpoint, this.m_testPidPositionSubsystem, this.m_testSingleMotorSubsystem)),
          Map.entry(PidMode.SpeedPidSubsystem, new InstantCommand(this.m_testPidSpeedSubsystem::decreaseSetpoint, this.m_testPidSpeedSubsystem, this.m_testSingleMotorSubsystem)),
          Map.entry(PidMode.PidCommand, new InstantCommand( ()-> {TestPIDCommand.decreaseSetpoint();} ))
      ), this::getMode));

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
