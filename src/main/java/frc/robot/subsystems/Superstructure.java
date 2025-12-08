package frc.robot.subsystems;

import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm.AbstractArm.ArmPosition;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.NewVision.VisionFuser;
import frc.robot.subsystems.NewVision.VisionFuser.VisionConstants;
import frc.robot.subsystems.NewDrive.*;
import frc.robot.subsystems.NewVision.Vision;
import frc.robot.Constants.ReefConstants;
import frc.robot.commands.DriveToPoseCommand;

public class Superstructure extends SubsystemBase {

    public final Arm arm;
    public final Intake intake;
    private final drive drive;
    private final Vision vision;
    private final VisionFuser visionFuser;

    private final CoralSensor coralsensor = new CoralSensor();

    public Pose2d goalPose2d;

    public boolean ifmechanismreverse;

    public int Level;

    public boolean intakemodel = false;

    private final DriveToPoseCommand driveToPoseCommand;

    public final HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(3.0, 0.0, 0.1), // X PID
            new PIDController(3.0, 0.0, 0.1), // Y PID
            new ProfiledPIDController(4.0, 0.0, 0.2,
                    new TrapezoidProfile.Constraints(Math.PI, Math.PI)) // θ PID
    );

    public Superstructure(Arm m_arm, Intake m_intake, drive drive, Vision vision, VisionFuser visionFuser) {
        this.arm = m_arm;
        this.intake = m_intake;
        this.drive = drive;
        this.vision = vision;
        this.visionFuser = visionFuser;
        controller.setTolerance(
                new Pose2d(0.25, 0.25, new Rotation2d(2)));// 「到達目標」的允許誤差範圍
        this.driveToPoseCommand = new DriveToPoseCommand(
                drive,
                this::alageautoAligngoal, // Supplier<Pose2d>
                drive::getPose // Supplier<Pose2d>
        );

    }

    public int tagId() {
        if (vision.getLeftTagId() != -1) {
            return vision.getLeftTagId();
        } else if (vision.getRightTagId() != -1) {
            return vision.getRightTagId();
        } else if (visionFuser.apriltagId() != -1) {
            return visionFuser.apriltagId();
        } else {
            return -1;
        }
    }

    public Pose2d alageautoAligngoal() {
        return ReefConstants.REEF_alage.get(this.tagId());
    }

    public Pose2d LeftautoAligngoal() {
        return ReefConstants.REEF_Left.get(this.tagId());
    }

    public Pose2d RightautoAligngoal() {
        return ReefConstants.REEF_Right.get(this.tagId());
    }

    public Pose2d ifreverse(Pose2d goalpPose2d) {
        Rotation2d currentRot = this.drive.getPose().getRotation(); // 機器人目前角度
        Rotation2d targetRotFlipped = goalpPose2d.getRotation().plus(Rotation2d.fromDegrees(180));

        // 計算兩個角度差的絕對值
        double diffNormal = Math.abs(currentRot.minus(goalpPose2d.getRotation()).getDegrees());
        double diffFlipped = Math.abs(currentRot.minus(targetRotFlipped).getDegrees());

        if (diffNormal <= diffFlipped) {
            this.ifmechanismreverse = false;
            return goalpPose2d;
        } else {
            this.ifmechanismreverse = true;
            return new Pose2d(
                    goalpPose2d.getX(),
                    goalpPose2d.getY(),
                    goalpPose2d.getRotation().plus(Rotation2d.fromDegrees(180)));
        }
    }

    public Command alageautoAlign() {
        // 1. 先執行一個瞬間動作：更新 goalPose2d
        return Commands.runOnce(() -> {
            Pose2d goal = alageautoAligngoal();
            if (goal == null) {
                goal = getalageClosestReefTagId();
            }
            // 這裡會更新全域變數 goalPose2d 以及 ifmechanismreverse
            goalPose2d = ifreverse(goal);
        })
                // 2. 更新完後，接著執行移動 (因為 driveToPoseCommand 是讀取 Supplier，所以會讀到最新的值)
                .andThen(new DriveToPoseCommand(
                        drive,
                        () -> goalPose2d, // 使用成員變數
                        drive::getPose));
    }

    public Command LeftautoAlign() {
        return Commands.runOnce(() -> {
            Pose2d goal = LeftautoAligngoal();
            if (goal == null) {
                goal = getLeftClosestReefTagId();
            }
            goalPose2d = ifreverse(goal);
        }).andThen(new DriveToPoseCommand(
                drive,
                () -> goalPose2d, // 使用成員變數
                drive::getPose));
    }

    public Command RightautoAlign() {
        return Commands.runOnce(() -> {
            Pose2d goal = RightautoAligngoal();
            if (goal == null) {
                goal = getRightClosestReefTagId();
            }
            goalPose2d = ifreverse(goal);
        }).andThen(new DriveToPoseCommand(
                drive,
                () -> goalPose2d, // 使用成員變數
                drive::getPose));
    }

    public Command LeftautochoserAlign() {
        return Commands.either(Leftreef(), this.alageautoAlign(), () -> this.coralsensor.isCoralIn());
    }

    public Command RightautochoserAlign() {
        return Commands.either(Rightreef(), this.alageautoAlign(), () -> this.coralsensor.isCoralIn());
    }

    public Command Leftreef() {
        return Commands.parallel(this.Levelreef(), this.LeftautoAlign());
    }

    public Command Rightreef() {
        return Commands.parallel(this.Levelreef(), this.RightautoAlign());
    }
    public Command Alagereef(){
        return Commands.parallel(this.Levelreef(), this.alageautoAlign());
    }

    public Pose2d getalageClosestReefTagId() {
        double minDistance = Double.POSITIVE_INFINITY; // 初始設為無限大
        int closestId = -1; // 預設沒找到

        for (Map.Entry<Integer, Pose2d> entry : ReefConstants.REEF_alage.entrySet()) {
            Pose2d tagPose = entry.getValue();
            double distance = this.drive.getPose().getTranslation().getDistance(tagPose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                closestId = entry.getKey();
            }
        }

        return ReefConstants.REEF_alage.get(closestId);
    }

    public Pose2d getLeftClosestReefTagId() {
        double minDistance = Double.POSITIVE_INFINITY; // 初始設為無限大
        int closestId = -1; // 預設沒找到

        for (Map.Entry<Integer, Pose2d> entry : ReefConstants.REEF_Left.entrySet()) {
            Pose2d tagPose = entry.getValue();
            double distance = this.drive.getPose().getTranslation().getDistance(tagPose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                closestId = entry.getKey();
            }
        }

        return ReefConstants.REEF_Left.get(closestId);
    }

    public Pose2d getRightClosestReefTagId() {
        double minDistance = Double.POSITIVE_INFINITY; // 初始設為無限大
        int closestId = -1; // 預設沒找到

        for (Map.Entry<Integer, Pose2d> entry : ReefConstants.REEF_Right.entrySet()) {
            Pose2d tagPose = entry.getValue();
            double distance = this.drive.getPose().getTranslation().getDistance(tagPose.getTranslation());

            if (distance < minDistance) {
                minDistance = distance;
                closestId = entry.getKey();
            }
        }

        return ReefConstants.REEF_Right.get(closestId);
    }

    public boolean ifclosegoal() {
        if (this.drive.getPose().getTranslation().getDistance(this.goalPose2d.getTranslation()) < 0.35) {
            return true;
        } else {
            return false;
        }
    }

    public void intakemodel() {
        this.intakemodel = (intakemodel) ? false : true;
    }

    public Command setintakemodel() {
        return Commands.runOnce(() -> this.intakemodel());
    }

    public Command intake() {
        return Commands.either(alageintake(), coralintake(), () -> intakemodel);
    }

    public Command intakestop() {
        return Commands.either(alageintakestop(), coralintakestop(), () -> intakemodel);
    }

    public Command alageintake() {
        return Commands.either(
                this.alageintakestop(),
                Commands.parallel(
                        this.arm.goToPosition(Arm.Positions.GROUND_ALGAE),
                        this.intake.inCoral().until(() -> this.intake.isalagein()).andThen(this.intake.keepAlgae())),
                () -> this.intake.isalagein());
    }

    public Command coralintake() {
        return Commands.either(
                this.coralintakestop(),
                Commands.parallel(
                        this.arm.goToPosition(Arm.Positions.GROUND_CORAL),
                        this.intake.inCoral().until(() -> this.coralsensor.isCoralIn()).andThen(this.intake.stop())),
                () -> this.coralsensor.isCoralIn());
    }

    public Command coralintakestop() {
        return Commands.either(Commands.parallel(this.arm.goToPosition(Arm.Positions.CORAL_STOW), this.intake.stop()),
                Commands.parallel(this.arm.goToPosition(Arm.Positions.STOW), this.intake.stop()),
                () -> this.coralsensor.isCoralIn());
    }

    public Command alageintakestop() {
        return Commands.either(Commands.parallel(this.arm.goToPosition(Arm.Positions.STOW), this.intake.keepAlgae()),
                Commands.parallel(this.arm.goToPosition(Arm.Positions.STOW), this.intake.stop()),
                () -> this.intake.isalagein());
    }

    public void setlevel(int level) {
        this.Level = level;
    }

    public Command setlevelCommand(int level) {
        return Commands.runOnce(() -> setlevel(level));
    }

    public ArmPosition LevelPosition() {
        switch (Level) {
            case 1:
                return Arm.Positions.L1;
            case 2:
                return ifmechanismreverse ? Arm.Positions.L2_OPP : Arm.Positions.L2;
            case 3:
                return ifmechanismreverse ? Arm.Positions.L3_OPP : Arm.Positions.L3;
            case 4:
                return ifmechanismreverse ? Arm.Positions.L4_OPP : Arm.Positions.L4;
            default:
                return ifmechanismreverse ? Arm.Positions.L4_OPP : Arm.Positions.L4;
        }
    }


public Command Levelreef() {
    return Commands.sequence(
        new WaitUntilCommand(() -> ifclosegoal()),
        
        Commands.defer(() -> {
            return this.arm.goToPosition(LevelPosition());
        }, Set.of((Subsystem) arm)) // 🛠️ 修正：加上 (Subsystem) 強制轉型
    );
}
public Command alagereef() {
    return Commands.sequence(
        new WaitUntilCommand(() -> ifclosegoal()),
        
        Commands.defer(() -> {
            return this.arm.goToPosition(LevelPosition());
        }, Set.of((Subsystem) arm)) // 🛠️ 修正：加上 (Subsystem) 強制轉型
    );
}

    public Command put() {
        return Commands.either(putcoral(), putalage(), () -> this.coralsensor.isCoralIn());
    }

    public Command putcoral() {
        return Commands.either(this.intake.outCoralReverse(), this.intake.outCoral(), () -> ifmechanismreverse);
    }

    public Command STOW() {
        return Commands.parallel(this.arm.goToPosition(Arm.Positions.STOW), this.intake.stop());
    }

    public Command putalage() {
        return Commands.either(this.intake.outAlgae(), this.intake.outAlgae(), () -> ifmechanismreverse);
    }

    // 假設你已經建立好 HolonomicDriveController
    // HolonomicDriveController controller = new
    // HolonomicDriveController(xController, yController, thetaController);

    public void drivetopose(Supplier<Pose2d> goalpose) {
        Pose2d currentPose = drive.getPose();
        Pose2d targetPose = goalpose.get();

        // 建立假的 Trajectory.State（因為只有一個目標 Pose）
        Trajectory.State fakeState = new Trajectory.State(
                0.0, 0.0, 0.0,
                targetPose,
                0.0);

        // 使用 HONPID 控制器計算速度
        ChassisSpeeds speeds = controller.calculate(currentPose, fakeState, targetPose.getRotation());

        // 發送速度給底盤
        drive.runVelocity(speeds);
    }

    public boolean atReference() {
        return controller.atReference();
    }

    // public Command driveToPoseCommand(Supplier<Pose2d> goalpose) {
    // return Commands
    // .sequence(Commands.run(() -> this.drivetopose(goalpose), this).until(() ->
    // this.atReference()));
    // }

    @Override
    public void periodic() {
        if (goalPose2d != null) {
            double[] pose = {
                    goalPose2d.getX(),
                    goalPose2d.getY(),
                    goalPose2d.getRotation().getRadians()
            };
            SmartDashboard.putNumberArray("goalPose", pose);
        }
        SmartDashboard.putNumber("tagid", this.tagId());
        SmartDashboard.putNumber("Level", Level);
        SmartDashboard.putBoolean("ifmechanismreverse", ifmechanismreverse);
    }
}