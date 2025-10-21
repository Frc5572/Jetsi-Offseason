package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.IntArrayList;
import frc.lib.util.Tuples;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIO.CameraInputs[] cameraInputs;
    private final RobotState state;
    private final Transform3d[] robotToCamera;
    public boolean seesMulitag = false;

  public Vision(RobotState state, Function<Constants.Vision.CameraConstants[], VisionIO> io) {
    super("Vision");
    this.state = state;
    this.io = io.apply(Constants.Vision.cameras);
    this.robotToCamera = Stream.of(Constants.Vision.cameras).map(x -> x.robotToCamera())
      .toArray(Transform3d[]::new);
    cameraInputs = new VisionIO.CameraInputs[Constants.Vision.cameras.length];
    for (int i = 0; i < Constants.Vision.cameras.length; i++) {
      cameraInputs[i] = new VisionIO.CameraInputs();
    }
  }

  private final IntArrayList tmpArrList = new IntArrayList();

  public Vision(VisionIO.Empty empty) {
  }

  @Override
  public void periodic() {
    io.updateInputs(cameraInputs);
    for (int i = 0; i < cameraInputs.length; i++) {
      Logger.processInputs("Camera" + i, cameraInputs[i]);
    }

    List<Tuples.Tuple3<Integer, Transform3d, PhotonPipelineResult>> results = new ArrayList<>();
    for (int i = 0; i < cameraInputs.length; i++) {
      var transform = robotToCamera[i];

      for (int j = 0; j < cameraInputs[i].results.length; j++) {
        var result = cameraInputs[i].results[j];
        results.add(new Tuples.Tuple3<>(i, transform, result));
      }
    }

    results.sort(
      (a, b) -> Double.compare(a._2().getTimestampSeconds(), b._2().getTimestampSeconds()));
    for (var result: results) {
      if (result._2().multitagResult.isPresent()) {
        seesMulitag = true;
      } else if (result._0() == 0) {
        seesMulitag = false;
      }
      state.addVisionObservation(result._2(), result._1(), result._0());
    }
  }
}
