// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import frc.robot.util.Swerve.ModuleLimits;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Degrees;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

public final class Constants {

    public static final class ModuleConstants {
        public static final double WilliamConstant = 1.042;
        public static final double kDrivingMotorFreeSpeedRps = MotorConstants.kFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.10068;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
        public static final double kDrivingMotorReduction = 5.95 * WilliamConstant;
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;
        public static final double kTurningGearRaitio = 1 / 19.6;
    }

    public static final class DriveConstants {

        public static final double kMaxSpeedMeterPerSecond = 2;
        public static final double kMaxAngularSpeedRadiansPerSecond = 3 * 1.8 * Math.PI;

        public static final double kMaxAccerationUnitsPerSecond = 5;

        public static final int[] kDriveMotorID = { 1, 2, 3, 4 };
        public static final int[] kTurnMotorID = { 5, 6, 7, 8 };
        public static final int[] kCANcoderID = { 9, 10, 11, 12 };

        public static final Translation2d[] moduleLocations = new Translation2d[] {
                new Translation2d(0.278, 0.278),
                new Translation2d(0.278, -0.278),
                new Translation2d(-0.278, 0.278),
                new Translation2d(-0.278, -0.278)
        };

        public static final ModuleLimits moduleLimitsFree = new ModuleLimits(kMaxSpeedMeterPerSecond,
                kMaxAccerationUnitsPerSecond, Units.degreesToRadians(1080.0));
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final int[] kOperatorReefSelecterButtonId = {
                11, 12, 16, 15, 14, 13, 10, 9, 5, 6, 7, 8
        };

        public static final double kDeadband = 0.06;
    }

    public static final class MotorConstants {
        public static final double kFreeSpeedRpm = 6784;
    }

    public static final class robotConstants {
        public static final double bumper = 10.0;
    }

    public static final class ElevatorConstants {
        public static final Per<AngleUnit, DistanceUnit> MOTOR_ROTATIONS_PER_METER_UNIT = Rotations.of(1)
                .div(Inches.of(13.0 / 50.0 * (Math.PI * 1.4397) * 2));// div馬達圈數/電梯移動的距離
        public static final double MOTOR_ROTATIONS_PER_METER = MOTOR_ROTATIONS_PER_METER_UNIT.in(Rotations.per(Meter));// 其實就是捲捲毛算給我的

        public static final Distance MIN_LENGTH = Inches.of(27.0);
        public static final Distance MIN_PADDED_LENGTH = MIN_LENGTH.plus(Inches.of(0.5));// 在最小長度的基礎上，加上 0.5 英吋作為安全墊
        public static final Distance MAX_LENGTH = Inches.of(66.0);
        public static final double MIN_LENGTH_ROTATIONS = MIN_LENGTH.in(Meters)
                * MOTOR_ROTATIONS_PER_METER;
        public static final double MAX_LENGTH_ROTATIONS = MAX_LENGTH.in(Meters)
                * MOTOR_ROTATIONS_PER_METER;

    }

    public static final class WristConstants {
        public static final double K_S = 0;
        public static final double K_V = 4.548;
        public static final double K_A = 0.2 * 0.45 / 0.25;
        public static final double MOTOR_ROTATIONS_PER_ARM_ROTATION = 48.0 / 9.0 * 40.0 / 15.0 * 40.0 / 15.0;
        public static final Angle CCW_LIMIT = Degrees.of(146.8);
        public static final Angle CW_LIMIT = Degrees.of(-70);
        public static final Angle K_G_ANGLE = Degrees.of(35.06);// Rotations.of(-0.072);
        public static final Angle K_G_ANGLE_WITH_CORAL = Degrees.of(45);
        public static final double K_G = 0.45;

    }
}
