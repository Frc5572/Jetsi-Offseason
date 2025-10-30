package frc.robot.subsystems.swerve;

import java.util.Queue;
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

@AutoLog
public class Timestamps {

    public double[] timestamps = new double[Constants.Swerve.MAX_ODOMETRY_SUBTICK_MEASUREMENTS];
    public int length = 0;

    public void fromQueue(Queue<Double> queue) {
        if (queue.isEmpty()) {
            timestamps[0] = Timer.getTimestamp();
            length = 1;
            return;
        }
        for (int i = 0; i < Constants.Swerve.MAX_ODOMETRY_SUBTICK_MEASUREMENTS; i++) {
            if (queue.isEmpty()) {
                break;
            }
            timestamps[i] = queue.poll();
            length = i + 1;
        }
    }

}
