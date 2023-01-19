package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
public class limelight {
    
public static void test(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
    
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 18.0;
    
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 30.0;
    
    // distance from the target to the floor
    double goalHeightInches = 60.0;
    
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    
    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
    System.out.println(distanceFromLimelightToGoalInches);
//return distanceFromLimelightToGoalInches;
    }
}
