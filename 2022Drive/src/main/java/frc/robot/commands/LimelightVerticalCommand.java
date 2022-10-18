// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.ObjectOutput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.LimelightVisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LimelightVerticalCommand extends PIDCommand {
  /** Creates a new LimelightVerticalCommand. */
  private static double kP = 1.0/2; // maximum
  private static double kI = 1;
  private static double kD = 1;

  private static double shootangle = 10;
  private DriveTrainSubsystem m_driveTrainSubsystem;
  private LimelightVisionSubsystem m_limelightVisionSubsystem;

  public LimelightVerticalCommand(
    DriveTrainSubsystem driveTrainSubsystem, 
    LimelightVisionSubsystem limelightVisionSubsystem ) {
      this(driveTrainSubsystem, limelightVisionSubsystem, shootangle);
    }
  public LimelightVerticalCommand(DriveTrainSubsystem driveTrainSubsystem, 
  LimelightVisionSubsystem limelightVisionSubsystem, 
  double targetHorizontalAngle) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        // If the vision system detects a target, return the vertical angle
        // or else return 0
        () -> {
          if (limelightVisionSubsystem.hasTarget()) { 
            System.out.println("horiziontal: " + limelightVisionSubsystem.getHorizontalAngle());
            return limelightVisionSubsystem.getHorizontalAngle();
          } else { 
            return 0;
          }
        },
        // This should return the setpoint (can also be a constant)
        () -> {
          System.out.println("Output target angle: " + targetHorizontalAngle);
          return targetHorizontalAngle;
        },
        // This uses the output
        output -> {
          // Use the output here
          //System.out.println("limelight-output" + ", "+output);
          System.out.println("Output: " + output);
          driveTrainSubsystem.tankDrive(output, -output, .4);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
    addRequirements(limelightVisionSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(10); // I(not me tho) think this is one degree
    //sam changed based on behaver test befor doing enting 
  }

  public double computeProcessVariable() { 
    if (this.m_limelightVisionSubsystem.hasTarget()) { 
      return this.m_limelightVisionSubsystem.getHorizontalAngle();
    } else { 
      return 0.0;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("is Finished = " + this.m_controller.atSetpoint());
   return true;
  }
}
