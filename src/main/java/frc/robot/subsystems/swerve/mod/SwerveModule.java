package frc.robot.subsystems.swerve.mod;

import org.littletonrobotics.junction.Logger;

public class SwerveModule {

    private final String inputsName;
    private final SwerveModuleIO io;
    private final SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();

    public SwerveModule(int moduleId, SwerveModuleIO io) {
        this.inputsName = "Swerve/Module" + moduleId;
        this.io = io;
    }

    public void updateInputs() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs(this.inputsName, this.inputs);
    }

    public void periodic() {

    }

}
