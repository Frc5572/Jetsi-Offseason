package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

    @AutoLog
    public static class ModuleInputs {
        public boolean driveConnected;
        public double drivePositionRad;
        public double driveVelocityRadPerSec;
        public double driveAppliedVolts;

        public boolean angleConnected;
        public double anglePositionRads;
        public double angleVelocityRadsPerSec;
        public double angleAppliedVolts;

        public boolean absoluteAngleConnected;
        public double angleAbsolutePositionRads;

        public double[] odometryDrivePositionsRad = new double[20];
        public double[] odometryTurnPositionsRad = new double[20];
    }

}
