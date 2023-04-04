package frc.robot;

public class MathUtil {
    public static double deadband(double speed, double deadband) {
        if (-deadband <= speed && speed <= deadband) {
            return 0;
        }
        else {
            return speed;
        }
    }
    public static double slope(double x1, double x2, double y1, double y2) {
        return (y2-y1)/(x2-x1);
    }
}
