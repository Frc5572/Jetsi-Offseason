package frc.robot.subsystems.swerve;


import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.swerve.SwerveModule;
import frc.lib.util.swerve.SwerveModuleReal;
import frc.robot.Constants;

/** Real Class for Swerve */
public class SwerveReal implements SwerveIO {

    private AHRS gyro = new AHRS(Constants.Swerve.navXID);

    /** Real Swerve Initializer */
    public SwerveReal() {}

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.yaw = gyro.getYaw();
        inputs.pitch = gyro.getPitch();
        inputs.roll = gyro.getRoll();

    }

    public SwerveModule createSwerveModule(int moduleNumber) {
        // Automatically fetch the values from the array we built using the module index
        int driveMotorID = Constants.Swerve.moduleConstants[moduleNumber].driveMotorId;
        int angleMotorID = Constants.Swerve.moduleConstants[moduleNumber].angleMotorId;
        int cancoderID = Constants.Swerve.moduleConstants[moduleNumber].canCoderId;
        Rotation2d angleOffset = Constants.Swerve.moduleConstants[moduleNumber].angleOffset;

        // Note: Passing moduleNumber first to match your updated SwerveModuleReal constructor!
        return new SwerveModule(moduleNumber, angleOffset, new SwerveModuleReal(moduleNumber,
            driveMotorID, angleMotorID, cancoderID, angleOffset));
    }

    @Override
    public SwerveModule[] createModules() {
        return new SwerveModule[] {createSwerveModule(0), // Front Left
            createSwerveModule(1), // Front Right
            createSwerveModule(2), // Back Left
            createSwerveModule(3) // Back Right
        };
    }

}
