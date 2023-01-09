// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//THIS IS HORIZONTEAL
package frc.robot.commands;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowLimelightSequence extends SequentialCommandGroup {
  /** Creates a new FollowLimelightSequence. */
  private DriveTrainSubsystem tankDriveSubsystem;
  private LimelightVisionSubsystem limelightVisionSubsystem;


  public FollowLimelightSequence(
    DriveTrainSubsystem tankDriveSubsystem,
    LimelightVisionSubsystem limelightVisionSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.tankDriveSubsystem = tankDriveSubsystem;
    this.limelightVisionSubsystem = limelightVisionSubsystem;
 
    this.addRequirements(this.tankDriveSubsystem, this.limelightVisionSubsystem);
    this.addCommands(
    //new InstantCommand(()-> this.m_limelightVisionSubsystem.turnOnLed()),
    new LimelightVerticalCommand(this.tankDriveSubsystem, this.limelightVisionSubsystem)
   // horizontalCenter(),
    //verticalCenter()
    //horizontalCenter()
    );
  } 
  //public FollowLimelightPidCommand horizontalCenter(){
  //return new FollowLimelightPidCommand(m_tankDriveSubsystem, m_limelightVisionSubsystem);

  public LimelightVerticalCommand verticalCenter(){
    return new LimelightVerticalCommand(this.tankDriveSubsystem, this.limelightVisionSubsystem);
    }
}

