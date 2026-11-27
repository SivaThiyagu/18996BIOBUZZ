package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

public class ManualShooter {
    private static final String[] MOTOR_NAMES = {};
    private static final DcMotor.Direction[] MOTOR_DIRECTIONS = {};
    private static final double SHOOTER_POWER = 0.0;

    private final List<DcMotor> motors = new ArrayList<>();
    private final String status;

    public ManualShooter(HardwareMap hardwareMap) {
        if (MOTOR_NAMES.length == 0 || MOTOR_NAMES.length != MOTOR_DIRECTIONS.length
                || !(SHOOTER_POWER > 0 && SHOOTER_POWER <= 1)) {
            status = "Disabled (config)";
            return;
        }

        for (int i = 0; i < MOTOR_NAMES.length; i++) {
            String name = MOTOR_NAMES[i];
            DcMotor motor = hardwareMap.tryGet(DcMotor.class, name);
            if (motor == null) {
                motors.clear();
                status = "Disabled: missing " + name;
                return;
            }
            motor.setPower(0);
            motor.setDirection(MOTOR_DIRECTIONS[i]);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motors.add(motor);
        }
        status = "Ready";
    }

    public void setRunning(boolean running) {
        for (DcMotor motor : motors) {
            motor.setPower(running ? SHOOTER_POWER : 0);
        }
    }

    public void stop() {
        setRunning(false);
    }

    public boolean isReady() {
        return !motors.isEmpty();
    }

    public String getStatus(boolean held) {
        return held && isReady() ? "Running" : status;
    }
}
