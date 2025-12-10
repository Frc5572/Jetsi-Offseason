package frc.robot.subsystems.swerve.util;

import java.util.Arrays;
import java.util.EnumSet;
import org.ejml.data.DMatrix3;
import org.ejml.dense.fixed.CommonOps_DDF3;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class SwerveRateLimiter {

    private double forwardLimit = Constants.Swerve.forwardLimit;
    private double forwardTiltLimit = Constants.Swerve.forwardTiltLimit;
    private double leftTiltLimit = Constants.Swerve.leftTiltLimit;
    private double rightTiltLimit = Constants.Swerve.rightTiltLimit;
    private double backTiltLimit = Constants.Swerve.backTiltLimit;
    private double skidLimit = Constants.Swerve.skidLimit;

    private final DoublePublisher[][] vecPublishers;
    private final DoublePublisher speedPublisher;

    public SwerveRateLimiter() {
        final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        vecPublishers = Arrays.stream(new String[] {"currentVel", "wantedVel", "wantedAcc"})
            .map((str) -> new DoublePublisher[] {
                ntInstance.getDoubleTopic("/SwerveRateLimiter/outputs/" + str + "/x").publish(),
                ntInstance.getDoubleTopic("/SwerveRateLimiter/outputs/" + str + "/y").publish(),
                ntInstance.getDoubleTopic("/SwerveRateLimiter/outputs/" + str + "/r").publish(),})
            .toArray(DoublePublisher[][]::new);
        speedPublisher =
            ntInstance.getDoubleTopic("/SwerveRateLimiter/outputs/wantedSpeed").publish();
        try {
            for (var item : SwerveRateLimiter.class.getDeclaredFields()) {
                if (item.getType().equals(double.class)) {
                    var topic = ntInstance.getDoubleTopic("/SwerveRateLimiter/" + item.getName());
                    var publisher = topic.publish();
                    var value = (double) item.get(this);
                    publisher.accept(value);
                    ntInstance.addListener(topic, EnumSet.of(Kind.kValueAll), (ev) -> {
                        try {
                            var newValue = ev.valueData.value.getDouble();
                            item.set(this, newValue);
                        } catch (IllegalArgumentException | IllegalAccessException e) {
                            e.printStackTrace();
                        }
                    });
                }
            }
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }

    public void update(ChassisSpeeds robotRelative) {
        currentVel.a1 = robotRelative.vxMetersPerSecond;
        currentVel.a2 = robotRelative.vyMetersPerSecond;
        currentVel.a3 = robotRelative.omegaRadiansPerSecond;
    }

    private final DMatrix3 currentVel = new DMatrix3();
    private final DMatrix3 wantedVel = new DMatrix3();
    private final DMatrix3 wantedAcc = new DMatrix3();

    private void publishVec(int id, DMatrix3 v) {
        vecPublishers[id][0].accept(v.a1);
        vecPublishers[id][1].accept(v.a2);
        vecPublishers[id][2].accept(v.a3);
    }

    public ChassisSpeeds limit(ChassisSpeeds wantedSpeedsRobotRelative) {
        double wantedSpeed = Math.hypot(wantedSpeedsRobotRelative.vxMetersPerSecond,
            wantedSpeedsRobotRelative.vyMetersPerSecond);

        wantedVel.a1 = wantedSpeedsRobotRelative.vxMetersPerSecond;
        wantedVel.a2 = wantedSpeedsRobotRelative.vyMetersPerSecond;
        wantedVel.a3 = wantedSpeedsRobotRelative.omegaRadiansPerSecond;

        CommonOps_DDF3.subtract(wantedVel, currentVel, wantedAcc);
        CommonOps_DDF3.divide(wantedAcc, 0.02);

        publishVec(0, currentVel);
        publishVec(1, wantedVel);
        publishVec(2, wantedAcc);
        speedPublisher.accept(wantedSpeed);

        // TODO
        return wantedSpeedsRobotRelative;
    }

}
