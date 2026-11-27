package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.automations.ColorSensorManager;

import java.util.Arrays;

@Disabled
@TeleOp
public class BareOpmode extends LinearOpMode {

    @Override
    public void runOpMode() {

        MotorEx transferMotor = new MotorEx(hardwareMap, HardwareMapNames.TRANSFER_MOTOR, Motor.GoBILDA.RPM_1150);

        waitForStart();

        while (opModeIsActive()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            transferMotor.set(power);
            telemetry.addData("Power", power);
            telemetry.update();
        }
    }
}
