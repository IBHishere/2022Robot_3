// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 *  Manage state of autonomous mode.  We found that 
 *
 * 
 */
public class AutonomousState {
    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private static NetworkTable table = inst.getTable("AutonomousState");

    private double priorCommandFinishPosition;
    private double priorCommandFinishAngle;

    private void logValues(){
        AutonomousState.table.getEntry("priorCommandFinishPosition").setDouble(priorCommandFinishPosition);
        AutonomousState.table.getEntry("priorCommandFinishAngle").setDouble(priorCommandFinishAngle);
    }

    public AutonomousState(DriveTrainSubsystem driveTrainSubsystem) {
        this.setValues(driveTrainSubsystem);
    }

    public double getPriorCommandFinishAngle() {
        return this.priorCommandFinishAngle;
    }

    public double getPriorCommandFinishPosition() {
        return this.priorCommandFinishPosition;
    }

    public void setValues(DriveTrainSubsystem driveTrainSubsystem) {
        this.priorCommandFinishPosition = driveTrainSubsystem.getPosition();
        this.priorCommandFinishAngle = driveTrainSubsystem.getAngle();
        this.logValues();
    }
}