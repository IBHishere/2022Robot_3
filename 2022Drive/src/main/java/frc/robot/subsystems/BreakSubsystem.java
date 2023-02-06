
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BreakSubsystem extends SubsystemBase {
  /** Creates a new BreakSubsystem. */
  private DoubleSolenoid Breakswitch = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
  private int breakState = 0;

  public BreakSubsystem() {
    Breakswitch.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void ToggleBreak() {
    breakState = breakState == 0 ? 1 : 0;
    if (breakState == 0) {
      Unbreak();
    } else if (breakState == 1) {
      Break();
    }
  }

  public void Break() {
    Breakswitch.set(DoubleSolenoid.Value.kForward);

  }

  public void Unbreak() {
    Breakswitch.set(DoubleSolenoid.Value.kReverse);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
