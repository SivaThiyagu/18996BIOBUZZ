package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Autonomous
public class PositionReset extends LinearOpMode {

    @Override
    public void runOpMode() {

        GlobalDataStorage.staticLocalizer = null;

        waitForStart();

        while (opModeIsActive()) {
            stop();
        }
    }
}
