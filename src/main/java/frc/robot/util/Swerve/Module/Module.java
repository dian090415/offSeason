package frc.robot.util.Swerve.Module;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Configs;
import frc.robot.Constants.DriveConstants;

public class Module {

    private final SparkFlex drive;
    private final SparkMax turn;

    private final RelativeEncoder driveEncoder, turnEncoder;
    private final SparkClosedLoopController driveController, turnController;

    private final CANcoder cancoder;

    private double angle;

    public Module(int ModuleId, boolean ModuleReverse) {

        this.drive = new SparkFlex(DriveConstants.kDriveMotorID[ModuleId], MotorType.kBrushless);
        this.turn = new SparkMax(DriveConstants.kTurnMotorID[ModuleId], MotorType.kBrushless);

        this.drive.configure(Configs.drivingConfig.inverted(ModuleReverse), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.turn.configure(Configs.turningConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.driveEncoder = drive.getEncoder();
        this.turnEncoder = turn.getEncoder();

        this.driveController = drive.getClosedLoopController();
        this.turnController = turn.getClosedLoopController();

        this.cancoder = new CANcoder(DriveConstants.kCANcoderID[ModuleId]);
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurnPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return turnEncoder.getVelocity();
    }

    public double getCANcoderRad() {
        angle = this.cancoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
        SmartDashboard.putNumber("absolutEncoderAngle", angle);
        return angle;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePosition(),
                Rotation2d.fromRadians(getTurnPosition()));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(),
                Rotation2d.fromRadians(getCANcoderRad()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState state = new SwerveModuleState();
        state.angle = Rotation2d.fromRadians(getTurnPosition());
        desiredState.optimize(state.angle);
        driveController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
        turnController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);
    }

    public void Stop() {
        driveController.setReference(0, ControlType.kVelocity);
        turnController.setReference(getTurnPosition(), ControlType.kPosition);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getCANcoderRad());
    }
}