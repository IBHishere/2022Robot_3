package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

  /** Creates a new ClimberSubsystem. */
  private static final double defaultSpeedLimit = 1;
  
  private CANSparkMax m_climber;
  private RelativeEncoder m_climberEncoder;
  
  private NetworkTableInstance inst; 
  private NetworkTable table; 
  
  
  public ClimberSubsystem(String logName, int motorSparkId, boolean isInverted) {
    this.inst = NetworkTableInstance.getDefault();
    this.table = inst.getTable("ClimberSubsystem"+logName);
    this.init(motorSparkId, isInverted);
  }
  
  private void init(int motorSparkId, boolean isInverted){
    this.m_climber = new CANSparkMax(motorSparkId, MotorType.kBrushless);// Constants.CLIMBER_MOTOR_CANLEFT_ID, MotorType.kBrushless);
    //m_climber.restoreFactoryDefaults();
    m_climber.setIdleMode(IdleMode.kBrake);
    m_climber.setInverted(isInverted);
   // m_climbGroup = new MotorControllerGroup(m_climberLeft,m_climberRight);
  
    m_climberEncoder= this.m_climber.getEncoder();
  }
  
  public double getPosition() {
    double pos = this.m_climberEncoder.getPosition() ; 
    this.table.getEntry("posclimber").setDouble(pos);
    return  pos;
  }

  public void climb(double speed) {
    this.climb(speed, defaultSpeedLimit);
  }

  public void climb(double speed, double speedLimit){
    speed = Math.signum(speed) * Math.min(speedLimit, Math.abs(speed));
    this.m_climber.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }    
}