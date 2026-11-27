package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public class TeamSwitch extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.circleWasPressed()) {
                GlobalDataStorage.autoTeam = Robot.Team.BLUE;
            } else if (gamepad1.squareWasPressed()) {
                GlobalDataStorage.autoTeam = Robot.Team.RED;
            }

            telemetry.addData("Team", GlobalDataStorage.autoTeam);
            telemetry.update();
        }
    }
}
