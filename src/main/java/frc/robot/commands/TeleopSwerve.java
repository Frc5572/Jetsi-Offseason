package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/**
 * Creates an command for driving the swerve drive during tele-op
 */
public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;

    private Swerve s_Swerve;
    private CommandXboxController controller;

    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, CommandXboxController controller, boolean fieldRelative,
        boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.controller = controller;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = -controller.getLeftY();
        double xAxis = -controller.getLeftX();
        double rAxis = -controller.getRightX();

        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        // if(controller.getRawButton(XboxController.Button.kX.value) && vision.getTargetFound()){
        // rotation = vision.getAimValue();
        // } else {
        // rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        // }

        rotation = rAxis * Constants.Swerve.maxAngularVelocity;


        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

    }

}
