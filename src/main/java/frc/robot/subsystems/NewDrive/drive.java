package frc.robot.subsystems.NewDrive;

import org.littletonrobotics.junction.Logger;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Robot;
import frc.robot.subsystems.NewVision.Vision;
import frc.robot.subsystems.NewVision.VisionFuser;
import frc.robot.util.Swerve.SwerveSetpoint;
import frc.robot.util.Swerve.SwerveSetpointGenerator;

public class drive extends SubsystemBase {

    private static drive autodrive;

    private final driveIO io;

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Field2d field;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.autoLocations);


    private final SwerveSetpointGenerator swerveSetpointGenerator;

    private SwerveSetpoint currentSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(),
            new SwerveModuleState[] {
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState(),
                    new SwerveModuleState()
            });

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    public drive(driveIO io) {

        this.io = io;

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
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }
    public SwerveDrivePoseEstimator poseEstimator(){
        return poseEstimator;
    }

        public static drive getinDrive(){
            if(autodrive == null){
                autodrive = new drive(null);
            }
                return autodrive;
            }

        public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = poseEstimator.getEstimatedPosition();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
        );

        // Apply the generated speeds
        autoVelocity(speeds);
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

    public void autoVelocity(ChassisSpeeds speeds) {

        ChassisSpeeds relativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.io.getRotation2d());
        ChassisSpeeds discretizeSpeeds = ChassisSpeeds.discretize(relativeSpeeds, Robot.kDefaultPeriod);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretizeSpeeds);
        this.io.setModuleStates(states);
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

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void addVisionMeasurement(Pose2d pose) {
        this.poseEstimator.addVisionMeasurement(pose, Timer.getFPGATimestamp());
    }
    public void OVaddVisionMeasurement(Pose2d pose , double timestampSeconds) {
        this.poseEstimator.addVisionMeasurement(pose, timestampSeconds);
    }


    @Override
    public void periodic() {

        // field.setRobotPose(poseEstimator.getEstimatedPosition());

        poseEstimator.update(
                io.getRotation2d(),
                this.io.getModulePositions());


        double[] pose = {poseEstimator.getEstimatedPosition().getX(), 
                        poseEstimator.getEstimatedPosition().getY(), 
                        Math.toRadians(io.getHeading())};


        SmartDashboard.putNumberArray("Pose", pose);

        SmartDashboard.putNumber("heading", io.getHeading());

        SmartDashboard.putNumber("turnRate", io.getTurnRate());
    }

}
