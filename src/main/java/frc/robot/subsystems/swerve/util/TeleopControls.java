package frc.robot.subsystems.swerve.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.jspecify.annotations.NullMarked;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;

/** Control scheme for teleoperated swerve */
@NullMarked
public class TeleopControls {

    /** Get desired Chassis Speeds (pseudo-field relative) from user inputs */
    public static Supplier<ChassisSpeeds> teleopControls(DoubleSupplier forward,
        DoubleSupplier right, DoubleSupplier turnCCW) {
        return () -> {
            double xaxis = right.getAsDouble();
            double yaxis = forward.getAsDouble();
            double raxis = turnCCW.getAsDouble();
            yaxis = MathUtil.applyDeadband(yaxis, Constants.DriverControls.stickDeadband);
            xaxis = MathUtil.applyDeadband(xaxis, Constants.DriverControls.stickDeadband);
            xaxis *= xaxis * Math.signum(xaxis);
            yaxis *= yaxis * Math.signum(yaxis);
            raxis = (Math.abs(raxis) < Constants.DriverControls.stickDeadband) ? 0 : raxis;
            return new ChassisSpeeds(yaxis * Constants.DriverControls.driverTranslationalMaxSpeed,
                xaxis * Constants.DriverControls.driverTranslationalMaxSpeed,
                raxis * Constants.DriverControls.driverRotationalMaxSpeed);
        };
    }

}
