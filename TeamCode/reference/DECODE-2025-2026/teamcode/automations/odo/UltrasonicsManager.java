package org.firstinspires.ftc.teamcode.automations.odo;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.Robot;

public class UltrasonicsManager {
    private final AnalogInput leftUltrasonic;
    private final AnalogInput rightUltrasonic;

    public UltrasonicsManager(HardwareMap hardwareMap) {
        leftUltrasonic = hardwareMap.get(AnalogInput.class, HardwareMapNames.LEFT_ULTRASONIC);
        rightUltrasonic = hardwareMap.get(AnalogInput.class, HardwareMapNames.RIGHT_ULTRASONIC);
    }

    public double getDistance() {
        if (GlobalDataStorage.team == Robot.Team.BLUE) {
            return getDistanceFromVoltage(rightUltrasonic.getVoltage()) - 4.8;
        } else if (GlobalDataStorage.team == Robot.Team.RED) {
            return getDistanceFromVoltage(leftUltrasonic.getVoltage()) - 4.8;
        } else {
            return Double.NaN;
        }
    }

    // Returns inches
    private static double getDistanceFromVoltage(double voltage) {
        return voltage / 3.3 * 520 / 2.54;
    }
}
