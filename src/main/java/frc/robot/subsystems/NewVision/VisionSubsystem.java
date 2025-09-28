package frc.robot.subsystems.NewVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable table1, table2;
    private double[] Pose1, Pose2;

    public VisionSubsystem() {
        this.table1 = NetworkTableInstance.getDefault().getTable("limelight-left");
        this.table2 = NetworkTableInstance.getDefault().getTable("limelight-right");
    }

    public Pose2d getPose1() {
        // return new Pose2d(Pose1[0], Pose1[1], Rotation2d.fromDegrees(Pose1[2]));

        try {
            return new Pose2d(Pose1[0], Pose1[1], Rotation2d.fromDegrees(Pose1[2]));
        } catch (Exception e) {
            return new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        }
    }

    public Pose2d getPose2() {
        try {
            return new Pose2d(Pose2[0], Pose2[2], Rotation2d.fromDegrees(Pose2[2]));
        } catch (Exception e) {
            return new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0));
        }
    }

    @Override
    public void periodic() {
        try {
            SmartDashboard.putNumberArray("Pose2", Pose2);
        } catch (Exception e) {
            // TODO: handle exception
        }


        try {
            this.Pose1 = table1.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        } catch (Exception e) {
            this.Pose1 = new double[6];// TODO: handle exception
        }

        try {
            this.Pose2 = table2.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
        } catch (Exception e) {
            this.Pose2 = new double[6];// TODO: handle exception
        }
    }

}
