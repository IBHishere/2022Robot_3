// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;

import javax.print.attribute.standard.MediaSizeName;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTestCommand extends CommandBase {
  /** Creates a new ShooterTestCommand. */
  
  private Date startTime = null;
  private double increseSpeedEverySoManyMS = 1000;
  private double currentIncrement = 0;
  private ShooterSubsystem shooterSubsystem;
  private double power = 0;
  private double powerIncrement = .1;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("ShooterTestCommand");
  
  public ShooterTestCommand(ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(shooterSubsystem);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if( this.startTime == null) this.startTime = new Date();

    Date currentTime = new Date();

    double msSinceStart = currentTime.getTime()- this.startTime.getTime() ;
    this.currentIncrement = 1 + Math.ceil(msSinceStart / this.increseSpeedEverySoManyMS);
    this.power = this.currentIncrement*this.powerIncrement;

    this.shooterSubsystem.runShooter(power);
    double velocity = this.shooterSubsystem.getVelocity();
    System.out.println("shooter, " + this.power + ", " + velocity);
    
    this.table.getEntry("power").setDouble(this.power);
    this.table.getEntry("velocity").setDouble(velocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if( this.power > 1.0 )
    {
      this.shooterSubsystem.stopShooter();
      return true;
    } else {
      return false;
    }
  }
}
