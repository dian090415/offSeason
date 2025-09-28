package frc.robot.subsystems.NewDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface driveIO {
    // gyro
    void zeroHeading();

    Rotation2d getRotation2d();

    double getHeading();

    double getTurnRate();

    // mooduleState
    SwerveModuleState[] getModuleStates();

    SwerveModulePosition[] getModulePositions();

    void setModuleStates(SwerveModuleState[] state);

    // encoders
    void resetEncoders();

    void stopModules();

}
