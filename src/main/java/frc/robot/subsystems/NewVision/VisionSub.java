package frc.robot.subsystems.NewVision;
// package frc.robot.subsystems.vision;

// import java.util.ArrayList;
// import java.util.List;

// import edu.wpi.first.math.Pair;
// import frc.robot.RobotState;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.littletonrobotics.junction.Logger;

// public class VisionSub extends SubsystemBase {

//     private List<Pair<VisionIO, VisionIOInputsAutoLogged>> inputsList = new ArrayList<>();

//     public VisionSub(VisionIO... visionIOList) {
//         for (VisionIO visionIO : visionIOList) {
//             inputsList.add(Pair.of(visionIO, new VisionIOInputsAutoLogged()));
//         }
//     }

//     @Override
//     public void periodic() {
//         List<RobotState.AprilTagObservation> aprilTagObservations = new ArrayList<>();
//         for (Pair<VisionIO, VisionIOInputsAutoLogged> camera : inputsList) {
//             var io = camera.getFirst();         // getIO
//             var inputs = camera.getSecond();    // getInputs

//             io.updateInputs(inputs);            // updateInputs

//             Logger.processInputs(io.getLimelightName(), inputs);    // logInputs

//             if (inputs.hasTarget && inputs.robotPoseBasedOffDistanceCalcAndTagLocation != null) { // 如果有target：


//                 aprilTagObservations.add(new RobotState.AprilTagObservation(
//                         io.getLimelightName(),
//                         io.getLimelightLocation(),
//                         inputs.tagId,
//                         inputs.robotPoseBasedOffDistanceCalcAndTagLocation));
                        

//             }
//         }

//         RobotState.getInstance()
//                 .addVisionObservation(aprilTagObservations.toArray(new RobotState.AprilTagObservation[0]));
//     }

//     public void setThrottleValue(int throttleValue) {
//         for (Pair<VisionIO, VisionIOInputsAutoLogged> camera : inputsList) {
//             var io = camera.getFirst();
//             io.setThrottleValue(throttleValue);
//         }
//     }
// }
