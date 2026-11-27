package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

public class TransferSubsystem extends SubsystemBase {
    private static final double TEST_POWER = 0.20;
    private final DcMotor motor;

    public TransferSubsystem(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, HardwareMapNames.TRANSFER_MOTOR);
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void move(boolean forward, boolean reverse) {
        motor.setPower(forward == reverse ? 0 : forward ? TEST_POWER : -TEST_POWER);
    }
    public void transfer() {
        motor.setPower(0.15);
        boolean up = true;
        motor.setTargetPosition(700);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(-0.15);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (motor.getCurrentPosition() > 700) {
            motor.setPower(-0.15);
            up = false;
        }

        if (motor.getCurrentPosition() < 25 && !up) {
            motor.setPower(0);
        }
    }

    public void reset() {
        motor.setTargetPosition(700);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.15);
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
