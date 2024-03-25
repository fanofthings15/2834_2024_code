package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable limelightTable;
    NetworkTableEntry tx, ty, ta, targetPose;

    public LimelightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
        targetPose = limelightTable.getEntry("targetpose_robotspace");
    }

    public double getApriltagX() {
        return tx.getDouble(0);
    }

    public double getApriltagY() {
        return ty.getDouble(0);
    }

    public double getApriltagArea() {
        return ta.getDouble(0);
    }
    
  public double getdistance() {
    double[] poseVals = targetPose.getDoubleArray(new double[6]);
    double dist = Units.metersToFeet(Math.sqrt(poseVals[1]*poseVals[1] + poseVals[2]*poseVals[2]));
    SmartDashboard.putNumber("Limelight Detected Distance", dist);
    return dist;
  }

  public boolean hasTarget() {
    return getApriltagArea() > 0;
  }
}
