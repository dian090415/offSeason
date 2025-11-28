package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewDrive.drive;

import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {
    private final drive drive;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;

    private final HolonomicDriveController controller;

    private static final double TRANSLATION_TOLERANCE = 0.05; // 5cm
    private static final double ROTATION_TOLERANCE = Math.toRadians(2.0); // 2度

    public DriveToPoseCommand(drive drive, Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> robotPoseSupplier) {
        this.drive = drive;
        this.targetPoseSupplier = targetPoseSupplier;
        this.robotPoseSupplier = robotPoseSupplier;

        // 建立 HolonomicDriveController（X, Y, θ）
        controller = new HolonomicDriveController(
            new PIDController(3.0, 0.0, 0.1), // X PID
            new PIDController(3.0, 0.0, 0.1), // Y PID
            new ProfiledPIDController(4.0, 0.0, 0.2,
                new TrapezoidProfile.Constraints(Math.PI, Math.PI)) // θ PID
        );

        controller.setTolerance(
            new Pose2d(TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE, new Rotation2d(ROTATION_TOLERANCE))//「到達目標」的允許誤差範圍
        );

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // 不需要 reset()，這裡什麼都不用做
    }

    @Override
    public void execute() {
        Pose2d currentPose = robotPoseSupplier.get();
        Pose2d targetPose = targetPoseSupplier.get();

        // 建立假的 trajectory state（因為我們只有一個 target pose）
        Trajectory.State fakeState = new Trajectory.State(
            0.0, 0.0, 0.0,
            targetPose,
            0.0
        );

        // 控制器計算
        var speeds = controller.calculate(currentPose, fakeState, targetPose.getRotation());

        // 發送給底層 drive subsystem（一般是 swerve）
        drive.autorunVelocity(speeds);
    }

    @Override
    public boolean isFinished() {
        return controller.atReference();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
