package frc.robot.subsystems.NewDrive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.NewVision.Vision;
import frc.robot.util.Swerve.SwerveSetpoint;
import frc.robot.util.Swerve.SwerveSetpointGenerator;

public class drive extends SubsystemBase {

    private final driveIO io;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduleLocations);

    private final SwerveSetpointGenerator swerveSetpointGenerator;

    private final Vision vision;

    private Rotation2d yaw;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });

    public drive(driveIO io, Vision vision) {

        this.io = io;

        this.vision = vision;


        Pose2d initialPose = new Pose2d(0, 0, io.getRotation2d());

        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                io.getRotation2d(),
                this.io.getModulePositions(),
                initialPose);

        this.swerveSetpointGenerator = new SwerveSetpointGenerator(
                this.kinematics,
                DriveConstants.moduleLocations);

        this.field = new Field2d();



        io.zeroHeading();
        io.resetEncoders();
    }

    public void runVelocity(ChassisSpeeds speeds) {

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, io.getRotation2d());

        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(fieldSpeeds, 0.02);

        SwerveModuleState[] SetPointStatesUnoptimized = kinematics.toSwerveModuleStates(discreteSpeeds);

        currentSetpoint = swerveSetpointGenerator.generateSetpoint(
                DriveConstants.moduleLimitsFree,
                currentSetpoint,
                discreteSpeeds,
                0.02);

        SwerveModuleState[] setpointStates = currentSetpoint.moduleStates();

        Logger.recordOutput("Drive/SwerveStates/SetpointsUnoptimized", SetPointStatesUnoptimized);
        Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("Drive/SwerveChassisSpeeds/Setpoints", currentSetpoint.chassisSpeeds());

        this.io.setModuleStates(setpointStates);
    }

    public void autoAlign(double[] speeds) {
        this.runVelocity(new ChassisSpeeds(speeds[0], speeds[1], speeds[2]));
    }

    public void stop() {
        this.io.stopModules();
    }

    public void zeroHeading() {
        this.io.zeroHeading();
    }

    public Command resetHeading() {
        return run(() -> this.zeroHeading());

    }


    @Override
    public void periodic() {

        field.setRobotPose(poseEstimator.getEstimatedPosition());

        poseEstimator.update(
                io.getRotation2d().rotateBy(yaw),
                this.io.getModulePositions());

        poseEstimator.addVisionMeasurement(vision.getLeftPose(), Timer.getFPGATimestamp());
        poseEstimator.addVisionMeasurement(vision.getRightPose(), Timer.getFPGATimestamp());

        double[] pose = {poseEstimator.getEstimatedPosition().getX(), 
                        poseEstimator.getEstimatedPosition().getY(), 
                        Math.toRadians(io.getHeading())};


        SmartDashboard.putNumberArray("Pose", pose);

        SmartDashboard.putNumber("heading", io.getHeading());

        SmartDashboard.putNumber("turnRate", io.getTurnRate());
    }

}
