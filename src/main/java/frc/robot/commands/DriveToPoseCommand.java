package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NewDrive.drive;

import java.util.function.Supplier;

public class DriveToPoseCommand extends Command {
    private final drive drive;
    private final Supplier<Pose2d> targetPoseSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;

    // --- 調參區 ---
    private static final double kMaxVel = 3.5;
    private static final double kMaxAccel = 2.5;
    
    private static final double kMaxAngularVel = Math.PI * 2;
    private static final double kMaxAngularAccel = Math.PI * 2;

    // ✅ 修改 PID 參數：這裡需要一點 kP 來推動最後的誤差
    // 直線 PID (Input: 距離誤差, Output: 速度補償)
    private final PIDController driveController = new PIDController(1.5, 0.0, 0.0);
    
    private final ProfiledPIDController thetaController = 
        new ProfiledPIDController(4.0, 0.0, 0.1, 
        new TrapezoidProfile.Constraints(kMaxAngularVel, kMaxAngularAccel));

    private final TrapezoidProfile driveProfile;
    private TrapezoidProfile.State lastProfiledState = new TrapezoidProfile.State(0, 0);
    
    private double lastTime = 0.0;

    private static final double TRANSLATION_TOLERANCE = 0.03;
    private static final double ROTATION_TOLERANCE = Math.toRadians(2.0);

    public DriveToPoseCommand(drive drive, Supplier<Pose2d> targetPoseSupplier, Supplier<Pose2d> robotPoseSupplier) {
        this.drive = drive;
        this.targetPoseSupplier = targetPoseSupplier;
        this.robotPoseSupplier = robotPoseSupplier;

        driveProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel));

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.setTolerance(ROTATION_TOLERANCE);
        driveController.setTolerance(TRANSLATION_TOLERANCE);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = robotPoseSupplier.get();
        Pose2d targetPose = targetPoseSupplier.get();
        
        double currentDist = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        
        // 繼承當前速度 (加負號，因為距離在縮短)
        // 使用 Math.min 保護，避免初始速度誤判導致 Profile 算錯
        double initialVelocity = -this.drive.getVelocity();
        
        lastProfiledState = new TrapezoidProfile.State(currentDist, initialVelocity);
    
        driveController.reset();
        thetaController.reset(currentPose.getRotation().getRadians());
        
        lastTime = Timer.getTimestamp();
    }

    @Override
    public void execute() {
        double currentTime = Timer.getTimestamp();
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        Pose2d currentPose = robotPoseSupplier.get();
        Pose2d targetPose = targetPoseSupplier.get();

        // --- 1. 計算直線運動 ---
        double distanceToTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        // Profile 計算 (Feedforward)
        var profileState = driveProfile.calculate(
            dt,
            new TrapezoidProfile.State(distanceToTarget, lastProfiledState.velocity),
            new TrapezoidProfile.State(0.0, 0.0)
        );
        
        if (distanceToTarget < TRANSLATION_TOLERANCE) {
            profileState = new TrapezoidProfile.State(0, 0);
        }

        lastProfiledState = profileState;

        // ✅ 加入 PID 補償 (Feedback)
        // 邏輯：目標是讓距離 (distanceToTarget) 變成 0
        // calculate(測量值, 目標值) -> calculate(distanceToTarget, 0)
        // 因為距離是正的，PID 算出來會是負值 (代表要減少距離)，我們取絕對值拿到「補償速率大小」
        double feedforward = Math.abs(profileState.velocity);
        double feedback = Math.abs(driveController.calculate(distanceToTarget, 0.0));
        
        // 最終速率 = Profile規劃的速率 + PID看到的誤差補償
        double targetSpeed = feedforward + feedback;

        // --- 2. 分解向量 ---
        Translation2d difference = targetPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d angleToTarget = difference.getAngle();
        
        double xSpeed = targetSpeed * angleToTarget.getCos();
        double ySpeed = targetSpeed * angleToTarget.getSin();

        // --- 3. 計算旋轉 ---
        double thetaSpeed = thetaController.calculate(
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians()
        );

        // --- 4. 發送指令 ---
        // ⚠️ 重要修改：這裡直接發送 "場地相對速度" (Field Relative Speeds)
        // 我們不在此處呼叫 fromFieldRelativeSpeeds，而是交給 Subsystem 處理
        // 或者如果你的 Subsystem 需要 Robot Relative，請確認 Subsystem 裡面有沒有重複轉換
        
        drive.autorunVelocity(
            new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = robotPoseSupplier.get();
        Pose2d targetPose = targetPoseSupplier.get();
        double distError = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        return distError < TRANSLATION_TOLERANCE && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}