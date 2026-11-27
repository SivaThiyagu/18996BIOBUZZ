package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.teamcode.HardwareMapNames;

@TeleOp
public class ServoTEst extends LinearOpMode {

    @Override
    public void runOpMode() {

        ServoEx servoEx = new ServoEx(hardwareMap, HardwareMapNames.SHOOTER_SERVO);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.squareWasPressed()) {
                servoEx.set(0);
            }

            if (gamepad1.circleWasPressed()) {
                servoEx.set(1);
            }
        }
    }
}
