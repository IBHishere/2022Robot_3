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

public class AutoCommand extends CommandBase {
  /** Creates a new AutoCommand. */
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private ShooterSubsystem m_shooterSubsystem;
  private IntakeSubsystem m_intakeSubsystem;

  public AutoCommand( VisionSubsystem visionSubsystem, DriveTrainSubsystem tankDriveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem){
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem, visionSubsystem, tankDriveSubsystem, shooterSubsystem );
    this.m_driveTrainSubsystem = tankDriveSubsystem;
    this.m_visionSubsystem = visionSubsystem;
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_intakeSubsystem = intakeSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
this.m_intakeSubsystem.intakePull();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_driveTrainSubsystem.tankDrive(0.5, -0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
