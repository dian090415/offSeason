package frc.robot.subsystems.Arm;

import frc.robot.subsystems.MainPivotS;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Head;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

public class Arm extends AbstractArm {
    public MainPivotS mainPivotS = new MainPivotS();
    public Elevator elevatorS = new Elevator();
    public Head wristS = new Head();
    private static final Distance SAFE_PIVOT_ELEVATOR_LENGTH = ArmPosition.SAFE_PIVOT_ELEVATOR_LENGTH;
    private static final Distance MIN_ELEVATOR_LENGTH = ElevatorConstants.MIN_LENGTH;

    public Trigger elevatorRetractedEnough = new Trigger(
            () -> elevatorS.getLengthMeters() < SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters) + Units.inchesToMeters(10));

    public Arm() {
        mainPivotS.setLengthSupplier(elevatorS::getLengthMeters);
        elevatorS.setAngleSupplier(mainPivotS::getAngleRadians);
        wristS.setMainAngleSupplier(mainPivotS::getAngleRadians);
    }

    public void update() {
        position = new ArmPosition(
                Radians.of(mainPivotS.getAngleRadians()),
                Meters.of(elevatorS.getLengthMeters()),
                Radians.of(wristS.getAngleRadians()));
    }

    public ArmPosition getPosition() {
        return position;
    }

    private Command goToPositionWithoutTuckCheck(ArmPosition position) {
        // 目標位置
        double positionPivotRadians = position.pivotRadians();
        double positionElevatorMeters = position.elevatorMeters();
        double positionWristRadians = position.wristRadians();

        // pivot 動之前，電梯最多只能伸到 SAFE 長度
        double postPivotElevator = MathUtil.clamp(
                positionElevatorMeters,
                MIN_ELEVATOR_LENGTH.in(Meters),
                SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters));

        Command command = either(
                // 情況 1：如果 pivot 角度已經差不多正確 → 直接去目標
                goDirectlyTo(positionPivotRadians, positionElevatorMeters, positionWristRadians),

                // 情況 2：如果 pivot 差很多 → 分三步安全移動
                sequence(
                        // (1) 先收電梯到安全長度，保持 pivot
                        goDirectlyToPivotHold(postPivotElevator, positionWristRadians)
                                .until(elevatorRetractedEnough),

                        // (2) pivot 移動到目標附近（誤差 < 10°）
                        goDirectlyTo(positionPivotRadians, postPivotElevator, positionWristRadians)
                                .until(() -> Math.abs(mainPivotS.getAngleRadians() - positionPivotRadians) < Units
                                        .degreesToRadians(10)),

                        // (3) 再把 elevator 和 wrist 伸到目標位置
                        goDirectlyTo(positionPivotRadians, positionElevatorMeters, positionWristRadians)),

                // 判斷條件：如果 pivot 角度跟目標差 < 6° → 用「情況 1」
                () -> Math.abs(positionPivotRadians - mainPivotS.getAngleRadians()) < Units.degreesToRadians(6));

        return command;
    }

    private Command goDirectlyTo(double mainPivotRadians, double elevatorMeters, double wristRadians) {
        return parallel(
                mainPivotS.goTo(() -> mainPivotRadians),
                elevatorDirectlyTo(elevatorMeters),
                wristDirectlyTo(wristRadians, elevatorMeters, () -> mainPivotRadians));
    }

    private Command elevatorDirectlyTo(double elevatorMeters) {
        return elevatorS.goToLength(
                () -> {
                    var dontHitDrivetrainTarget = elevatorMeters;
                    return MathUtil.clamp(
                            dontHitDrivetrainTarget,
                            ElevatorConstants.MIN_LENGTH.in(Meters),

                            ElevatorConstants.MAX_LENGTH.in(Meters));
                });
    }

    private Command wristDirectlyTo(double wristRadians, double elevatorSetpointMeters,
            DoubleSupplier mainPivotRadians) {
        return wristS.goTo(
                () -> {
                    double dontGoIntoBumperMinLimit = mainPivotRadians.getAsDouble() < Units.degreesToRadians(17)
                            || mainPivotS.getAngleRadians() < Units.degreesToRadians(17)
                                    ? 0
                                    : WristConstants.CW_LIMIT.in(Radians);
                    double dontGoIntoElevatorMaxLimit = Math.min(elevatorS.getLengthMeters(),
                            elevatorSetpointMeters) < ElevatorConstants.MAX_LENGTH.in(Meters) - Units.inchesToMeters(6)
                                    ? WristConstants.CCW_LIMIT.in(Radians) - Units.degreesToRadians(25)
                                    : WristConstants.CCW_LIMIT.in(Radians);
                    return MathUtil.clamp(wristRadians, dontGoIntoBumperMinLimit, dontGoIntoElevatorMaxLimit);
                });
    }

    private Command goDirectlyToPivotHold(

            double elevatorMeters, double wristRadians) {
        return parallel(
                mainPivotS.hold(),
                elevatorDirectlyTo(elevatorMeters),
                wristDirectlyTo(wristRadians, elevatorMeters, mainPivotS::getAngleRadians));
    }

    public Command goToPosition(ArmPosition position) {
        if (position.pivotRadians() < Units.degreesToRadians(17)) {
            double shrunkenElevator = MathUtil.clamp(position.elevatorMeters(), MIN_ELEVATOR_LENGTH.in(Meters),
                    SAFE_PIVOT_ELEVATOR_LENGTH.in(Meters));
            Angle wristSafeToLower = position.wristAngle().lt(Degrees.of(0)) ? Degrees.of(0) : position.wristAngle();
            return goToPositionWithoutTuckCheck(
                    new ArmPosition(Degrees.of(17), Meters.of(shrunkenElevator), wristSafeToLower))
                    .until(() -> wristS.getAngleRadians() > Units.degreesToRadians(-3))
                    .unless(() -> wristS.getAngleRadians() > Units.degreesToRadians(-3))
                    .andThen(goToPositionWithoutTuckCheck(position));
        } else {
            return goToPositionWithoutTuckCheck(position);
        }
    }
//       @Override
//   public Command processorWithHome() {

//     return sequence(
//         goToPosition(Arm.Positions.SCORE_PROCESSOR)
//     .until(
//     ()->this.position.withinTolerance(Arm.Positions.SCORE_PROCESSOR,
//     Units.degreesToRadians(2), Units.inchesToMeters(0.5),
//     Units.degreesToRadians(3600)
//     )),
//     parallel(
//     new ScheduleCommand(
//     wristS.driveToHome()
//     ),
//     new ScheduleCommand(
//       elevatorDirectlyTo(Arm.Positions.SCORE_PROCESSOR.elevatorMeters())
//       ),
//       new ScheduleCommand(
//         mainPivotS.goTo(Arm.Positions.SCORE_PROCESSOR::pivotRadians)
//         )
//     ));
//   }
}
