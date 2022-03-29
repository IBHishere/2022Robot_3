// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class WindClimberCommand extends CommandBase {
  /** Creates a new WindClimberCommand. */
  private DoubleSupplier m_windAmount;
  private ClimberSubsystem m_climberSubsystem;
  private final double maxWindSpeed = .2;
  
  public WindClimberCommand( ClimberSubsystem climberSubsystem, 
    DoubleSupplier windAmount
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberSubsystem);
    this.m_climberSubsystem = climberSubsystem;
    this.m_windAmount = windAmount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.m_climberSubsystem.climb( this.m_windAmount.getAsDouble()*this.maxWindSpeed);
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
