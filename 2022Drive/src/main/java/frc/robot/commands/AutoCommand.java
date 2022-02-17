// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutoCommand extends CommandBase {
  /** Creates a new AutoCommand. */
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;
  private double m_leftDrive = 0;
  private double m_rightDrive = 1;


  public AutoCommand( VisionSubsystem visionSubsystem, DriveTrainSubsystem tankDriveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, visionSubsystem, tankDriveSubsystem, shooterSubsystem );
    
    this.m_driveTrainSubsystem = tankDriveSubsystem;
    this.m_visionSubsystem = visionSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;

  }

  @Override
  public void initialize() {
  disableShooter();
  this.m_intakeSubsystem.intakePull();
  autoDrive(1,-1);
  this.m_shooterSubsystem.startQueue2();
  Timer.delay(2.0); 
  autoDrive(0,0);
  Timer.delay(3.0); 
  this.m_intakeSubsystem.intakePush();
  this.m_shooterSubsystem.stopQueue2();
  autoDrive(0.5, 0.5);
  this.m_shooterSubsystem.startQueue();
  Timer.delay(5.0);
  robotOff();
  }
  public void autoDrive(double setLeft, double setRight ){
    this.m_leftDrive = setLeft;
    this.m_rightDrive = setRight;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_driveTrainSubsystem.autoTankDrive(this.m_leftDrive, this.m_rightDrive);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
 // the following commands are groupings for efficiency
 public void disableShooter(){
  this.m_shooterSubsystem.stopShooter();
  this.m_shooterSubsystem.stopQueue();
  this.m_shooterSubsystem.stopQueue2();
}
public void disableintake(){
  this.m_intakeSubsystem.intakeStop();
}
public void robotOff(){
  disableShooter();
  disableintake();
  autoDrive(0, 0);
}

}
