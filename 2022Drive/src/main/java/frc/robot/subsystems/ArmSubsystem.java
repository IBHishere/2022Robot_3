// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;
// import java.lang.Object;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.lang.AutoCloseable;
// import frc.robot.Constants;
// import com.playingwithfusion.TimeOfFlight;


// public class ArmSubsystem extends SubsystemBase {
//   /** Creates a new ArmSubsystem. */

//   private TimeOfFlight armDistance;

//   public ArmSubsystem() {
//     armDistance = new TimeOfFlight(Constants.TIMEOFFLIGHT_ID);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public double getArmDistance() {
//     double distance = armDistance.getRange();
//     System.out.println(distance);
//     return distance;
//    }
// }
