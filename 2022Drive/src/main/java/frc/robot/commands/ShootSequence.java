// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.BeltSubsystem;
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
  private BeltSubsystem m_beltSubsystem;
  public ShootSequence(ShooterSubsystem shooterSubsystem, BeltSubsystem beltSubsystem) {
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_beltSubsystem = beltSubsystem;
    addRequirements(m_shooterSubsystem);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      ShootSequence1()
     );
  }
  public SequentialCommandGroup ShootSequence1(){

    //TODO: we have taken off the feeder wheel because the gearbox was not working well.  
    System.out.println("ShootSequence1");
//     return new InstantCommand(()-> this.m_shooterSubsystem.startShooter() )
//        .andThen(new WaitCommand(5))
//        .andThen(()-> this.m_shooterSubsystem.stopShooter())
// ;
     return 
     //new WaitCommand(.001)
     //.andThen(
         new InstantCommand (()-> this.m_shooterSubsystem.startShooter() )
     .andThen( new WaitCommand(4))
     .andThen( new InstantCommand( ()-> this.m_beltSubsystem.startBelt(1.0) ))
     .andThen( new InstantCommand( () -> this.m_shooterSubsystem.startQueueFeederWheel()))
     .andThen( new WaitCommand(1.5))
     .andThen( new InstantCommand( ()-> this.m_shooterSubsystem.stopShooter() ))
     .andThen( new InstantCommand( ()-> this.m_beltSubsystem.stopBelt() ))
    .andThen( ()-> this.m_shooterSubsystem.stopQueueFeederWheel() )
    ;
    }
     public SequentialCommandGroup onlyShoot(){
       return new InstantCommand (()-> this.m_shooterSubsystem.startShooter() ).andThen(new WaitCommand(3).andThen(new InstantCommand(()-> this.m_shooterSubsystem.startShooter())));

     }
}

