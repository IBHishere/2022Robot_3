// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class PidSettings {
    public double kP;
    public double kI;
    public double kD;
    
    public PidSettings(double kPP, double kII, double kDD) {
        this.kP = kPP;
        this.kI = kII;
        this.kD = kDD;
    }
}
// comment
