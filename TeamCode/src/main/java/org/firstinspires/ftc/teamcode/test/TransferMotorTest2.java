package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

@TeleOp(name = "BIOBUZZ transfer Motor Test", group = "Tests")
public class TransferMotorTest2 extends LinearOpMode {
    private static final double TEST_POWER = 0.25;

    @Override
    public void runOpMode() {
        DcMotor transfer = hardwareMap.get(DcMotor.class, HardwareMapNames.TRANSFER_MOTOR);
        transfer.setPower(0);
        transfer.setDirection(DcMotor.Direction.REVERSE);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            waitForStart();
            while (opModeIsActive()) {
                transfer.setPower(0.2);
                telemetry.addData("Tick Position", transfer.getCurrentPosition());
                telemetry.update();
            }
        }
    }

