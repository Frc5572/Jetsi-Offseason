package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class LimeLight extends SubsystemBase {
    RobotState state;
    LimeLightIO io;
    LimeLightInputsAutoLogged inputs = new LimeLightInputsAutoLogged();

    public LimeLight(RobotState state, LimeLightIO io) {
        this.state = state;
        this.io = io;
    }

    @Override
    public void periodic() {
        for (int i = 0; i > 1; i++) {
            state.addVisionObservations(inputs.cameraPose[i]);
        }
        Logger.processInputs("LL/", inputs);
    }
}
