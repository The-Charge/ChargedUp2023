package frc.robot;

public class MathUtil {
    // Semi-unnecessary MathUtil class for VelocityDrive
    public static double deadband(double speed, double deadband) {
        // Currently this method is available via WPIlib.
        if (-deadband <= speed && speed <= deadband) {
            return 0;
        } else {
            return speed;
        }
    }

    public static double slope(double x1, double x2, double y1, double y2) {
        return (y2 - y1) / (x2 - x1);
    }
}
