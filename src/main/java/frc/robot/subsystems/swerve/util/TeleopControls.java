package frc.robot.subsystems.swerve.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

/** Control scheme for teleoperated swerve */
public class TeleopControls {

    /** Get desired Chassis Speeds (pseudo-field relative) from user inputs */
    public static Supplier<ChassisSpeeds> teleopControls(DoubleSupplier forward,
        DoubleSupplier right, DoubleSupplier turnCCW) {
        return () -> {
            double xaxis = right.getAsDouble();
            double yaxis = forward.getAsDouble();
            double raxis = turnCCW.getAsDouble();
            yaxis = MathUtil.applyDeadband(yaxis, Constants.STICK_DEADBAND);
            xaxis = MathUtil.applyDeadband(xaxis, Constants.STICK_DEADBAND);
            xaxis *= xaxis * Math.signum(xaxis);
            yaxis *= yaxis * Math.signum(yaxis);
            raxis = (Math.abs(raxis) < Constants.STICK_DEADBAND) ? 0 : raxis;
            return new ChassisSpeeds(yaxis * Constants.Swerve.maxSpeed,
                xaxis * Constants.Swerve.maxSpeed, raxis * Constants.Swerve.maxAngularVelocity);
        };
    }

}
