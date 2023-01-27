package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;
public class Limelight {
// how many degrees back is your limelight rotated from perfectly vertical?
    private static final double limelightMountAngleDegrees = 0;
  // distance  from the center of the Limelight lens to the floor
    private static final  double limelightLensHeightInches = 31.0; 
  // distance from the target to the floor
    private static final double goalHeightInches = 15.0;


    public double getDistance() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        
        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);
        System.out.println(distanceFromLimelightToGoalInches);

        return distanceFromLimelightToGoalInches;
    }
    public double getXOffset() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        
        double targetOffsetAngle_Horizontal = tx.getDouble(0.0);

        return targetOffsetAngle_Horizontal;
    }
}
