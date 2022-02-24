// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.subsystems.DriveTrainSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class PIDTurnRobotCommand extends PIDCommand {
//   private double m_targetAngle;
//   private DriveTrainSubsystem m_driveTrainSubsystem;
//   private static double kP = 1;
//   private static double kI = 5;
//   private static double kD = 0;
// private double targetAngle;


//   /** Creates a new PIDturnrobot. */
  // public PIDTurnRobotCommand(DriveTrainSubsystem driveTrainSubsystem, double targetAngle) {
    // super(
    //     // The controller that the command will use
    //     new PIDController(kP, kI, kD),
    //     // This should return the measurement
    //     () -> driveTrainSubsystem.getAngle(),
    //     // This should return the setpoint (can also be a constant)
    //     () -> targetAngle,
    //     // This uses the output
    //     output -> {
    //       driveTrainSubsystem.tankDrive(output, -output);
    //     });

//         this.targetAngle = targetAngle;
//         this.m_targetAngle = targetAngle;
//         this.m_driveTrainSubsystem = driveTrainSubsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(driveTrainSubsystem);
//     // Configure additional PID options by calling `getController` here.
    
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return this.m_controller.atSetpoint();
//   }
// }
