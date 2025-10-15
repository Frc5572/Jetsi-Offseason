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
        io.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LL/", inputs);
        io.setRobotOrentation(state.getGlobalPoseEstimate());
        if (inputs.tagCountOne > 0) {
            state.addVisionObservations(inputs.cameraPoseOne, inputs.tagCountOne, inputs.timestampOne);
        }
        if (inputs.tagCountTwo > 0) {
            state.addVisionObservations(inputs.cameraPoseTwo, inputs.tagCountTwo, inputs.timestampTwo);
        }
    }
}
