package frc.robot.subsystems.swerve.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveRateLimiter {

    private final double forwardLimit = Constants.Swerve.forwardLimit;
    private final double forwardTiltLimit = Constants.Swerve.forwardTiltLimit;
    private final double leftTiltLimit = Constants.Swerve.leftTiltLimit;
    private final double rightTiltLimit = Constants.Swerve.rightTiltLimit;
    private final double backTiltLimit = Constants.Swerve.backTiltLimit;
    private final double skidLimit = Constants.Swerve.skidLimit;

    public void update(SwerveModuleState[] states) {
        // TODO
    }

    public ChassisSpeeds limit(ChassisSpeeds robotRelative) {
        // TODO
        return robotRelative;
    }

}
