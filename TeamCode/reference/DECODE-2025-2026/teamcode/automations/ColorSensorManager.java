package org.firstinspires.ftc.teamcode.automations;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.SensorRevColorV3;

import java.lang.Math;

public class ColorSensorManager {
    private SensorRevColorV3 upper, middle;

    public enum Color {
        GREEN,
        PURPLE,
        NONE
    }

    public ColorSensorManager(HardwareMap hardwareMap) {
        upper = new SensorRevColorV3(hardwareMap, HardwareMapNames.UPPER_COLOR_SENSOR);
        middle = new SensorRevColorV3(hardwareMap, HardwareMapNames.MIDDLE_COLOR_SENSOR);
    }

    public boolean ballDetected() {
        return getUpperDistance() < 60 ||
                getMiddleDistance() < 38;
                //getColor(upper.getARGB()) != Color.NONE ||
                //getColor(middle.getARGB()) != Color.NONE;
    }

    public double getUpperDistance() {
        return upper.distance(DistanceUnit.MM);
    }

    public double getMiddleDistance() {
        return middle.distance(DistanceUnit.MM);
    }

    public int[] getUpperColor() {
        //return getColor(upper.getARGB());
        return upper.getARGB();
    }

    public int[] getMiddleColor() {
        //return getColor(middle.getARGB());
        return middle.getARGB();
    }

    // a r g b
    int[] purple = {100, 80, 120, 130};
    int[] green = {70, 50, 100, 70};
    int[] ambient = {80, 60, 100, 85};

    private Color getColor(int[] point) {
        double pd = calcDistance(point, purple);
        double gd = calcDistance(point, green);
        double ad = calcDistance(point, ambient);

        if (pd < ad) {
            if (pd < gd) {
                return Color.PURPLE;
            }
            else {
                return Color.GREEN;
            }
        }
        else {
            if (ad < gd) {
                return Color.NONE;
            }
            else {
                return Color.GREEN;
            }
        }
    }

    private double calcDistance(int[] point, int[] ref) {
        double a, r, g, b;
        a = point[0] - ref[0];
        r = point[1] - ref[1];
        g = point[2] - ref[2];
        b = point[3] - ref[3];

        double ds = a * a + r * r + g * g + b * b;
        return Math.pow(ds, 0.5);
    }
}
