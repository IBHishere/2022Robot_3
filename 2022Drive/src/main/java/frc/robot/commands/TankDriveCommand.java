// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TankDriveCommand extends CommandBase {
  /** Creates a new TankDriveCommand. */

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private DoubleSupplier m_getLeftY;
  private DoubleSupplier m_getRightX;

  public TankDriveCommand(DriveTrainSubsystem driveSubsystem, DoubleSupplier getLeftY, DoubleSupplier getRightX) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    this.m_driveTrainSubsystem = driveSubsystem;
    this.m_getLeftY = getLeftY;
    this.m_getRightX = getRightX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_driveTrainSubsystem.tankDrive(this.m_getLeftY.getAsDouble(), this.m_getRightX.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_driveTrainSubsystem.tankDrive(0.0, 0.0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
