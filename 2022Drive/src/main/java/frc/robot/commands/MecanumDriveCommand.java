

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
//find correct one later
import frc.robot.subsystems.MecanumDriveSubsystem;

public class MecanumDriveCommand extends CommandBase {
  /** Creates a new MecanumDriveCommand. */

  private MecanumDriveSubsystem m_mecanumDriveSubsystem;
  //Change this later
  private DoubleSupplier m_getLeftX;
  private DoubleSupplier m_getLeftY;
  private DoubleSupplier m_getRightX;
  private int countCall = 0;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("MecanumDriveCommand_1");

  public MecanumDriveCommand(MecanumDriveSubsystem mecanumSubsystem, DoubleSupplier getLeftX, DoubleSupplier getLeftY, DoubleSupplier getRightX) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mecanumSubsystem);
    
    this.table.getEntry("init").setString("done");
    this.m_mecanumDriveSubsystem = mecanumSubsystem;
    this.m_getLeftX = getLeftX;
    this.m_getLeftY = getLeftY;
    this.m_getRightX = getRightX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.table.getEntry("countCall").setNumber(this.countCall++);
    double test1 = this.m_getLeftY.getAsDouble();
    this.table.getEntry("m_getLeftY").setNumber(test1);
    
    this.m_mecanumDriveSubsystem.MecanumDrive(this.m_getLeftY.getAsDouble(), this.m_getLeftX.getAsDouble(), this.m_getRightX.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.m_mecanumDriveSubsystem.MecanumDrive(0.0, 0.0, 0.0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}