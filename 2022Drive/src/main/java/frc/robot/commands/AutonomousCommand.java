// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.AutonomousSubsystem;

// public class AutonomousCommand extends CommandBase {
//   /** Creates a new AutonomousCommand. */
//   private AutonomousSubsystem m_autonomousSubsystem;
//   public AutonomousCommand(AutonomousSubsystem subsystem) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(m_autonomousSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     this.m_autonomousSubsystem.tankDrive(0.2, 0.2);
//     // this is the starting value for testing
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
