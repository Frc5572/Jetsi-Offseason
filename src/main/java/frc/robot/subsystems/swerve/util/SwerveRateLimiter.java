package frc.robot.subsystems.swerve.util;

import java.util.EnumSet;
import org.ejml.data.DMatrix3;
import org.ejml.dense.fixed.CommonOps_DDF3;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

/**
 * Limiter for better swerve controls (based on presentation from 1690:
 * https://www.youtube.com/watch?v=vUtVXz7ebEE)
 */
public class SwerveRateLimiter {

    private double forwardLimit = Constants.Swerve.forwardLimit;
    private double forwardTiltLimit = Constants.Swerve.forwardTiltLimit;
    private double leftTiltLimit = Constants.Swerve.leftTiltLimit;
    private double rightTiltLimit = Constants.Swerve.rightTiltLimit;
    private double backTiltLimit = Constants.Swerve.backTiltLimit;
    private double skidLimit = Constants.Swerve.skidLimit;

    /** Limiter for better swerve controls */
    public SwerveRateLimiter() {
        final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
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

    /** Update current speed from odometry */
    public void update(ChassisSpeeds robotRelative) {
        currentVel.a1 = robotRelative.vxMetersPerSecond;
        currentVel.a2 = robotRelative.vyMetersPerSecond;
        currentVel.a3 = robotRelative.omegaRadiansPerSecond;
    }

    private final DMatrix3 currentVel = new DMatrix3();
    private final DMatrix3 wantedVel = new DMatrix3();
    private final DMatrix3 wantedAcc = new DMatrix3();

    private void publish(String name, DMatrix3 v) {
        Logger.recordOutput("SwerveRateLimiter/" + name, new Twist2d(v.a1, v.a2, v.a3));
    }

    private void publish(String name, double v) {
        Logger.recordOutput("SwerveRateLimiter/" + name, v);
    }

    /** Calculate new chassis speeds that satisfy all limits */
    public ChassisSpeeds limit(ChassisSpeeds wantedSpeedsRobotRelative) {
        double currentSpeed = Math.hypot(currentVel.a1, currentVel.a2);
        double wantedSpeed = Math.hypot(wantedSpeedsRobotRelative.vxMetersPerSecond,
            wantedSpeedsRobotRelative.vyMetersPerSecond);

        publish("currentSpeed", currentSpeed);
        publish("wantedSpeed", wantedSpeed);

        wantedVel.a1 = wantedSpeedsRobotRelative.vxMetersPerSecond;
        wantedVel.a2 = wantedSpeedsRobotRelative.vyMetersPerSecond;
        wantedVel.a3 = wantedSpeedsRobotRelative.omegaRadiansPerSecond;
        publish("wantedVel", wantedVel);

        CommonOps_DDF3.subtract(wantedVel, currentVel, wantedAcc);
        CommonOps_DDF3.divide(wantedAcc, 0.02);
        publish("wantedAccStep0", wantedAcc);

        // Step 1: Robot cannot accelerate indefinitely. Ensure accelerations are achievable
        // (sub-max to ensure correctness even under degradation).
        double subphysicalAccelerationLimit =
            Math.max(1.0 - (currentSpeed / Constants.Swerve.maxSpeed), 0.0);
        publish("subphysicalAccelerationLimit", subphysicalAccelerationLimit);
        double maxForwardAccel = forwardLimit * subphysicalAccelerationLimit;
        publish("maxForwardAccel", maxForwardAccel);

        // get acceleration in direction of current velocity
        double wantedAccMagnitude = wantedAcc.a1 * currentVel.a1 / currentSpeed
            + wantedAcc.a2 * currentVel.a2 / currentSpeed;
        publish("wantedAccMagnitudeStep1", wantedAccMagnitude);
        if (wantedAccMagnitude > maxForwardAccel) {
            double mul = maxForwardAccel / wantedAccMagnitude;
            wantedAcc.a1 *= mul;
            wantedAcc.a2 *= mul;
        }
        publish("wantedAccStep1", wantedAcc);

        // Step 2: Robot may accelerate too fast and result in tilting. Limit directional
        // acceleration to prevent this.
        if (wantedAcc.a1 > forwardTiltLimit) {
            wantedAcc.a1 = forwardLimit;
        }
        if (wantedAcc.a1 < -backTiltLimit) {
            wantedAcc.a1 = -backTiltLimit;
        }
        if (wantedAcc.a2 > leftTiltLimit) {
            wantedAcc.a2 = leftTiltLimit;
        }
        if (wantedAcc.a2 < -rightTiltLimit) {
            wantedAcc.a2 = -rightTiltLimit;
        }
        publish("wantedAccStep2", wantedAcc);

        // Step 3: Robot accelerating in a different direction may result in skidding. Limit
        // magnitude of acceleration to prevent this.
        wantedAccMagnitude = Math.hypot(wantedAcc.a1, wantedAcc.a2);
        publish("wantedAccMagnitudeStep3", wantedAccMagnitude);
        if (wantedAccMagnitude > skidLimit) {
            double multiplier = skidLimit / wantedAccMagnitude;
            wantedAcc.a1 *= multiplier;
            wantedAcc.a2 *= multiplier;
        }
        publish("wantedAccStep3", wantedAcc);

        CommonOps_DDF3.scale(0.02, wantedAcc);
        CommonOps_DDF3.add(currentVel, wantedAcc, wantedVel);

        publish("resultant", wantedVel);
        publish("resultantSpeed", Math.hypot(wantedVel.a1, wantedVel.a2));

        return new ChassisSpeeds(wantedVel.a1, wantedVel.a2, wantedVel.a3);
    }

}
