package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

@TeleOp(name = "rand test", group = "tests")
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor servo = hardwareMap.get(DcMotor.class, HardwareMapNames.SHOOTER_SERVO);
        servo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        servo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.xWasPressed()) {
                servo.setPower(0.1);
            }


        }
    }
}
