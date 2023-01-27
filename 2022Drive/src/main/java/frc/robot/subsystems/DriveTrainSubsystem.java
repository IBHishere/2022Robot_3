// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import frc.robot.Constants;


// import java.lang.Math;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMax.IdleMode;



// public class DriveTrainSubsystem extends SubsystemBase {

//   private DifferentialDrive m_myRobot;
//   private CANSparkMax m_leftMotor1;
//   private RelativeEncoder m_leftEncoder ;
//   private CANSparkMax m_rightMotor1;
//   private RelativeEncoder m_rightEncoder; 
//   private CANSparkMax m_leftMotor2;
//   private RelativeEncoder m_left2Encoder;
//   private CANSparkMax m_rightMotor2;
//   private RelativeEncoder m_right2Encoder ;
//   private NetworkTableInstance inst = NetworkTableInstance.getDefault();
//   private NetworkTable table = inst.getTable("DriveTrainSubsystem");
//   public final static double highSpeedLimit = .65;
//   public final static double lowSpeedLimit = .50;
//   private double currentSpeed = highSpeedLimit;  

//   private IdleMode m_idleMode = IdleMode.kCoast;
//   public boolean doLog = false;


//   /** Creates a new DriveTrain. */
//   public DriveTrainSubsystem() {
//     this.init();
//   }

//   public void setIdleMode(IdleMode idleMode) {
//     this.m_idleMode = idleMode;
//     m_leftMotor1.setIdleMode(this.m_idleMode);
//     m_leftMotor2.setIdleMode(this.m_idleMode);
//     m_rightMotor1.setIdleMode(this.m_idleMode);
//     m_rightMotor2.setIdleMode(this.m_idleMode);
    
//   }

//   public IdleMode getIdleMode() { return this.m_idleMode;}

//   private void init(){
//     m_leftMotor1 = new CANSparkMax(Constants.LEFT_MOTOR_CAN1_ID, MotorType.kBrushless);
//     m_leftMotor2 = new CANSparkMax(Constants.LEFT_MOTOR_CAN2_ID, MotorType.kBrushless);
//     m_leftMotor1.setInverted(true);
//     m_leftMotor2.setInverted(true);
//     MotorControllerGroup m_leftMotorGroup = new MotorControllerGroup(m_leftMotor1,m_leftMotor2);
    
//     m_rightMotor1 = new CANSparkMax(Constants.RIGHT_MOTOR_CAN1_ID, MotorType.kBrushless);
//     m_rightMotor2 = new CANSparkMax(Constants.RIGHT_MOTOR_CAN2_ID, MotorType.kBrushless);
//     m_rightMotor1.restoreFactoryDefaults();
//     m_rightMotor2.restoreFactoryDefaults();
//     MotorControllerGroup m_rightMotorGroup = new MotorControllerGroup(m_rightMotor1,m_rightMotor2);

//     m_leftEncoder= this.m_leftMotor1.getEncoder();
//     m_rightEncoder= this.m_rightMotor1.getEncoder();
//     m_left2Encoder = this.m_leftMotor2.getEncoder();
//     m_right2Encoder = this.m_rightMotor2.getEncoder();
    
//     zeroEncoders();
//     this.setIdleMode(IdleMode.kCoast);

//     m_myRobot = new DifferentialDrive(m_leftMotorGroup, m_rightMotorGroup);
//   }

//   public void setSpeed(double speed) {
//     this.currentSpeed = speed;
//   }


//   @Override
//   public void periodic() {}
//     // This method will be called once per scheduler run

//   private double computeActualDriveFromInput(double joystickValue, double speedScalingFactor) {
//     //TODO: Move this to constants?
//     double zeroLimit = 0.1;
//     double driveCurvePower = 5.0; // 3.0;
    
//     double outputValue  = Math.abs(joystickValue) < zeroLimit ? 0 :
//        Math.pow(joystickValue, driveCurvePower) * speedScalingFactor;

//     return outputValue;
//   }

//   public void tankDrive(double leftJoystickValue, double rightJoystickValue) {
//     this.tankDrive(leftJoystickValue, rightJoystickValue, this.currentSpeed);
//   }

//   public void tankDrive(
//     double leftJoystickValue, 
//     double rightJoystickValue, 
//     double speedScalingFactor ) {
    
    
//     if(doLog) System.out.println("joystick-l-r,  " + leftJoystickValue +", " + rightJoystickValue);

//     //limit input to [-1,1] range
//     leftJoystickValue = Math.signum(leftJoystickValue) * Math.min(1, Math.abs(leftJoystickValue));
//     rightJoystickValue = Math.signum(rightJoystickValue) * Math.min(1, Math.abs(rightJoystickValue));
    
    
//     //log joystick values to network tables
//     table.getEntry("motorLeftValue").setDouble(leftJoystickValue);
//     table.getEntry("motorRightValue").setDouble(rightJoystickValue);

//     //convert joystick values to motor inputs
//     double leftMotorValue = this.computeActualDriveFromInput(leftJoystickValue, speedScalingFactor);
//     double rightMotorValue = this.computeActualDriveFromInput(rightJoystickValue, speedScalingFactor);

//     table.getEntry("computedMotorLeftValue").setDouble(leftMotorValue);
//     table.getEntry("computedMotorRightValue").setDouble(rightMotorValue);

//     if(doLog) System.out.println("drive-l-r,  " + leftMotorValue +", " + rightMotorValue);

//     m_myRobot.tankDrive( leftMotorValue, rightMotorValue);
//   }

//   private double getLeftPosition() {
//     return (this.m_leftEncoder.getPosition() + this.m_left2Encoder.getPosition() )/2.0;
//   }


//   private double getRightPosition() {
//     return (this.m_rightEncoder.getPosition() + this.m_right2Encoder.getPosition() )/2.0;
//   }

//   public double getPosition() {
//       double pos = (this.getLeftPosition() + this.getRightPosition())/2.0; 
      
//       // System.out.println("Enc-l1-l2-r1-r2-l-r-p, " 
//       //   + this.m_leftEncoder.getPosition() + ", "
//       //   + this.m_left2Encoder.getPosition() + ", "
//       //   + this.m_rightEncoder.getPosition() + ", "
//       //   + this.m_right2Encoder.getPosition() + ", "
//       //   + this.getLeftPosition() + ", "
//       //   + this.getRightPosition() + ", "
//       //   + pos
//       // );

//       this.table.getEntry("pos").setDouble(pos);
//       return  pos;
//   }

//   public double getAngle() {
//       double angle = (this.getLeftPosition() - this.getRightPosition())/2.0; 
//       this.table.getEntry("angle").setDouble(angle);
//       return angle;
//   }

//   public void zeroEncoders() {
//     this.m_leftEncoder.setPosition(0);
//     this.m_left2Encoder.setPosition(0);
//     this.m_rightEncoder.setPosition(0);
//     this.m_right2Encoder.setPosition(0);
//   }
// }

