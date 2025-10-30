package frc.robot.util.math;

import edu.wpi.first.math.geometry.Twist2d;

public class Approx {

    public static boolean approxEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean approxEquals(double a, double b) {
        return approxEquals(a, b, 1e-9);
    }

    public static boolean approxEquals(Twist2d a, Twist2d b) {
        return approxEquals(a.dx, b.dx) && approxEquals(a.dy, b.dy)
            && approxEquals(a.dtheta, b.dtheta);
    }

}
