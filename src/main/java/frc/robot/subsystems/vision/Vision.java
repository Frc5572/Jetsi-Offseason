package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.util.SwerveState;
import frc.robot.util.Tuples.Tuple2;

/** Vision Subsystem */
public class Vision extends SubsystemBase {

    private final VisionIO io;
    private final VisionIO.CameraInputs[] cameraInputs;
    private final String[] cameraInputKeys;
    private final SwerveState state;

    /** Vision Subsystem */
    public Vision(SwerveState state, VisionIO io) {
        super("Vision");
        this.io = io;
        this.state = state;
        this.cameraInputs = IntStream.range(0, Constants.Vision.cameraConstants.length)
            .mapToObj((_x) -> new VisionIO.CameraInputs()).toArray(VisionIO.CameraInputs[]::new);
        this.cameraInputKeys = IntStream.range(0, Constants.Vision.cameraConstants.length)
            .mapToObj((i) -> "Vision/Camera_" + i).toArray(String[]::new);
    }

    @Override
    public void periodic() {
        io.updateInputs(cameraInputs);
        for (int i = 0; i < cameraInputs.length; i++) {
            Logger.processInputs(cameraInputKeys[i], cameraInputs[i]);
        }

        // Get 2-tuples of (camera idx, result)
        List<Tuple2<Integer, PhotonPipelineResult>> results = new ArrayList<>();
        for (int i = 0; i < cameraInputs.length; i++) {

            for (int j = 0; j < cameraInputs[i].results.length; j++) {
                var result = cameraInputs[i].results[j];
                results.add(new Tuple2<>(i, result));
            }
        }

        // Sort by timestamp (earlier pipeline results come first)
        results.sort(
            (a, b) -> Double.compare(a._1().getTimestampSeconds(), b._1().getTimestampSeconds()));

        for (var result : results) {
            state.addVisionObservation(Constants.Vision.cameraConstants[result._0()], result._1());
        }
    }

}
