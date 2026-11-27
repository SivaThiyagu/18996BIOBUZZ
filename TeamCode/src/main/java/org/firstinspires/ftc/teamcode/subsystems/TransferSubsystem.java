package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

public class TransferSubsystem {
    private static final double TEST_POWER = 0.20;
    private final DcMotor motor;

    public TransferSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, HardwareMapNames.TRANSFER_MOTOR);
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(boolean forward, boolean reverse) {
        motor.setPower(forward == reverse ? 0 : forward ? TEST_POWER : -TEST_POWER);
    }




    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public double getPower() {
        return motor.getPower();
    }

    public void stop() {
        motor.setPower(0);
    }
}
