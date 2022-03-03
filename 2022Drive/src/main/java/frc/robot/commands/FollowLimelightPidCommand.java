// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowLimelightPidCommand extends PIDCommand {
  private static double kP = 1.0/25.0; // maximum
  private static double kI = 0;
  private static double kD = 1;

  private DriveTrainSubsystem m_driveTrainSubsystem;
  private LimelightVisionSubsystem m_limelightVisionSubsystem;

  public FollowLimelightPidCommand(
    DriveTrainSubsystem driveTrainSubsystem, 
    LimelightVisionSubsystem limelightVisionSubsystem ) {
      this(driveTrainSubsystem, limelightVisionSubsystem, 0);
    }


  
  /** Creates a new FollowLimelightPidCommand. */
  public FollowLimelightPidCommand(
      DriveTrainSubsystem driveTrainSubsystem, 
      LimelightVisionSubsystem limelightVisionSubsystem, 
      double targetHorizontalAngle) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () -> { 
          return 
            	  limelightVisionSubsystem.hasTarget() ?  
                limelightVisionSubsystem.getHorizontAngle()
                : 10
                ; 
        },
        // This should return the setpoint (can also be a constant)
        () -> targetHorizontalAngle,
        // This uses the output
        output -> {
          System.out.println("limelight-output" + ", "+output);

          driveTrainSubsystem.tankDrive(output, -output, .3);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
    addRequirements(limelightVisionSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(10); // I think this is one degree
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // this.m_controller.atSetpoint();
  }
}
