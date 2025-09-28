package frc.robot.subsystems.NewVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    
    private final NetworkTable left, right;
    
    private double[] leftPose, rightPose;

    public Vision() {
        left = NetworkTableInstance.getDefault().getTable("limelight-left");
        right = NetworkTableInstance.getDefault().getTable("limelight-right");
    }

    public void getPoses() {
        if (left.getEntry("botpose_wpiblue").getDoubleArray(new double[6]) != null) {
            leftPose = left.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }
        if (right.getEntry("botpose_wpiblue").getDoubleArray(new double[6]) != null) {
            rightPose = right.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        }
    }

    public Pose2d getLeftPose() {
        if (leftPose != null) {
            return new Pose2d(leftPose[0], leftPose[1], Rotation2d.fromDegrees(leftPose[5]));
        } else {
            return new Pose2d();
        }
    }

    public Pose2d getRightPose() {
        if (rightPose != null) {
            return new Pose2d(rightPose[0], rightPose[1], Rotation2d.fromDegrees(rightPose[5]));
        } else {
            return new Pose2d();
        }
    }

    @Override
    public void periodic() {
        getPoses();
    }
}
