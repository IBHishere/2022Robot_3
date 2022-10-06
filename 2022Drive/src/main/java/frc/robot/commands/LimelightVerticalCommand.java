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
public class LimelightVerticalCommand extends PIDCommand {
  /** Creates a new LimelightVerticalCommand. */
  private static double kP = 1.0/25.0; // maximum
  private static double kI = 0;
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
  double targetVerticalAngle) {
    super(
        // The controller that the command will use
        new PIDController(kP, kI, kD),
        // This should return the measurement
        () ->  {
          return
        limelightVisionSubsystem.hasTarget() ?  
        limelightVisionSubsystem.getVerticalAngle()
        : 10;
        },
        // This should return the setpoint (can also be a constant)
        () -> targetVerticalAngle,
        // This uses the output
        output -> {
          // Use the output here
          //System.out.println("limelight-output" + ", "+output);

          driveTrainSubsystem.tankDrive(output, output, .25);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSubsystem);
    addRequirements(limelightVisionSubsystem);
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(40); // I(not me tho) think this is one degree
    //sam changed based on behaver test befor doing enting 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return this.m_controller.atSetpoint();
  }
}
