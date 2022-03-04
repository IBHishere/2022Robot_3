// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Timer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSequence extends SequentialCommandGroup {
  /** Creates a new ShootSequence. */
  private ShooterSubsystem m_shooterSubsystem;
  public ShootSequence(ShooterSubsystem ShooterSubsystem) {
    m_shooterSubsystem = ShooterSubsystem;
    addRequirements(m_shooterSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
   
    // ShootSequence0(),
    ShootSequence1()

    );
  }
  public SequentialCommandGroup ShootSequence1(){

    return new InstantCommand(
      ()->{
        this.m_shooterSubsystem.startShooter();
        

    }).andThen(new WaitCommand(1))
    .andThen(new InstantCommand(
      ()-> {
      this.m_shooterSubsystem.startQueue();
      this.m_shooterSubsystem.startQueue2();
      }
    ).andThen(new WaitCommand(1)).andThen( new InstantCommand(
      ()->{
    this.m_shooterSubsystem.stopShooter();
    this.m_shooterSubsystem.stopQueue();
      this.m_shooterSubsystem.stopQueue2();
       } )))
    ;
    }
  public SequentialCommandGroup ShootSequence0(){
  return new SequentialCommandGroup(
            // start shooter
            new InstantCommand( ()-> this.m_shooterSubsystem.startShooter(), this.m_shooterSubsystem ),
            new WaitCommand(1.5), // wait 1.5 to let the shooter spin up
            new InstantCommand( ()-> { 
              this.m_shooterSubsystem.startQueue2();
              this.m_shooterSubsystem.startQueue();
            }, this.m_shooterSubsystem ),
            new WaitCommand(1.0), // wait 1 s to complete the shot
            new InstantCommand( ()-> { 
              this.m_shooterSubsystem.stopShooter();
              this.m_shooterSubsystem.stopQueue2();
              this.m_shooterSubsystem.stopQueue();
            }, this.m_shooterSubsystem )
          );
        
  }
}

