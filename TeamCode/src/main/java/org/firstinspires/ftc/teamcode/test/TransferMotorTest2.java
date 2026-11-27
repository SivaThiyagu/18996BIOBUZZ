package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

@TeleOp(name = "BIOBUZZ transfer Motor Test", group = "Tests")
public class TransferMotorTest2 extends LinearOpMode {
    private static final double TEST_POWER = 0.25;

    @Override
    public void runOpMode() {
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);



            waitForStart();


            while (opModeIsActive()) {
                if (gamepad1.xWasPressed()) {
                    transfer.transfer();
                }

                telemetry.addData("Tick Position", transfer.getPosition());
                telemetry.update();
        }
    }}


