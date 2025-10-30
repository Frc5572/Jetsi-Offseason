package frc.robot.util.swerve;

public class CommonReductions {

    private CommonReductions() {}

    public static enum Mk4i {
        L1(8.14), L2(6.75), L3(6.12), TURN((150.0 / 7.0));

        public final double reduction;

        Mk4i(double reduction) {
            this.reduction = reduction;
        }

    }

}
