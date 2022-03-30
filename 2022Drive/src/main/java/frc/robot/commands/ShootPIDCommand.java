// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.LinearSetpointTrajectory;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPIDCommand extends PIDCommand {
  /** Creates a new ShootPIDCommand. */

  private static final double kP = 1/5000;
  private static final double kD = 0;
  private static final double kI = 0;
  public static final double HighGoalShooterSpeed = 5000;
  public static final double LowGoalShooterSpeed = 500;

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("ShootPIDCommand");

  public enum GoalMode {
    HighGoal,
    LowGoal
  }

  public static GoalMode CurrentGoalMode = GoalMode.HighGoal;
  public static void setGoalMode(GoalMode mode) { ShootPIDCommand.CurrentGoalMode = mode;}
  public static double getTargetVelocity() {
    return ShootPIDCommand.CurrentGoalMode == GoalMode.HighGoal ? 
      ShootPIDCommand.HighGoalShooterSpeed:
      ShootPIDCommand.LowGoalShooterSpeed;
  }

  private boolean m_doesItEverEnd;

  public ShootPIDCommand(
    double setpoint,
    ShooterSubsystem shooterSubsystem,
    boolean doesItEverEnd
  ) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> shooterSubsystem.getVelocity(),
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        output -> {
          ShootPIDCommand.table.getEntry("shooterVelocity").setDouble(shooterSubsystem.getVelocity());
          ShootPIDCommand.table.getEntry("output").setDouble(output);
          
          shooterSubsystem.runShooter(output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(shooterSubsystem);
    // Configure additional PID options by calling `getController` here.

    this.m_doesItEverEnd = doesItEverEnd;
    if( doesItEverEnd)  this.getController().setTolerance(10); //TODO: Extract constant
  }


  public ShootPIDCommand(
    LinearSetpointTrajectory setpointTrajectory, 
    ShooterSubsystem m_shooterSubsystem,
    boolean doesItEverEnd
    ) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> m_shooterSubsystem.getVelocity(),
        // This should return the setpoint (can also be a constant)
        () -> setpointTrajectory.getSetpoint(),
        // This uses the output
        output -> {
          System.out.println("shooter, " + m_shooterSubsystem.getVelocity() + ", " + output);   
          m_shooterSubsystem.increaseVelocity(output);
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(m_shooterSubsystem);
    // Configure additional PID options by calling `getController` here.

    this.m_doesItEverEnd = doesItEverEnd;
    if( doesItEverEnd)  this.getController().setTolerance(10); //TODO: Extract constant
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (this.m_doesItEverEnd) 
      return this.getController().atSetpoint();
    
    return false;
  }
public static void ToggleMode() {
  ShootPIDCommand.CurrentGoalMode = 
      ShootPIDCommand.CurrentGoalMode == GoalMode.HighGoal? 
      GoalMode.LowGoal : GoalMode.HighGoal;
}
}
