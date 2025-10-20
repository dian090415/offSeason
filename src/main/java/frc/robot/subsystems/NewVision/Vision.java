package frc.robot.subsystems.NewVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.NewDrive.drive;

public class Vision extends SubsystemBase {
    
    private final NetworkTable left, right;

    private final drive drive;
    
    private double[] leftPose, rightPose;

    public Vision(drive drive) {

        this.drive = drive;
        left = NetworkTableInstance.getDefault().getTable("limelight-left");
        right = NetworkTableInstance.getDefault().getTable("limelight-right");
    }

    public void getPoses() {
        if (left.getEntry("tv").getDouble(0) == 1) {
            leftPose = left.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        } else {
            leftPose = null;
        }
    
        if (right.getEntry("tv").getDouble(0) == 1) {
            rightPose = right.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        } else {
            rightPose = null;
        }
    }
    
    public void getLeftPose() {
        if (leftPose != null && leftPose.length >= 6) {
            drive.addVisionMeasurement(new Pose2d(
                leftPose[0],
                leftPose[1],
                Rotation2d.fromDegrees(leftPose[5])
            ));
        }
    }
    
    public void getRightPose() {
        if (rightPose != null && rightPose.length >= 6) {
            drive.addVisionMeasurement(new Pose2d(
                rightPose[0],
                rightPose[1],
                Rotation2d.fromDegrees(rightPose[5])
            ));
        }
    }

    @Override
    public void periodic() {

        try {
            getPoses();
            
        } catch (Exception e) {
            // TODO: handle exception
        }

        try {
            getLeftPose();
            getRightPose();
        } catch (Exception e) {
            // TODO: handle exception
        }
        
    }
}
