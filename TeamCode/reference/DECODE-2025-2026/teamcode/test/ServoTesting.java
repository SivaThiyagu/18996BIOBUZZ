package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.HardwareMapNames;

@TeleOp
@Disabled
public class ServoTesting extends LinearOpMode {
    @Override
    public void runOpMode() {

        ServoEx gateServo = new ServoEx(hardwareMap, HardwareMapNames.GATE_SERVO);

        waitForStart();

        while (opModeIsActive()) {
            gateServo.set(0.5);
        }
    }
}
