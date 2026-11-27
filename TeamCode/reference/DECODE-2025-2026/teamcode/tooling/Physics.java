package org.firstinspires.ftc.teamcode.tooling;


public class Physics {
    final static double g = 9.81;

    /***
     * CHECK THAT v0^4 - g(gd + 2hv^2) > 0 BEFORE CALLING
     * @param d distance
     * @param h height
     * @param v0 initial velocity
     * @return angle to horizontal
     */
    public static double determineTrajectoryAngle(double d, double h, double v0) {
        if (ErrorClassifier.assertStatement( () -> Math.pow(v0, 4) - g * (g * d + 2 * h * v0 * v0) > 0, "Shot impossible")) {
            return 0;
        }
        double numerator = v0 * v0 + Math.pow(Math.pow(v0, 4) - g * (g * d + 2 * h * v0 * v0), .5);
        double denominator = g * d;
        return Math.atan(numerator / denominator);
    }
}
