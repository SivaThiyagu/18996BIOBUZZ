package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.Constants;

@TeleOp
public class BasicProg extends LinearOpMode {
    private DcMotorEx motor;

    public void runOpMode() {

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"motor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int target_pos = motor.getCurrentPosition();;
        boolean down = false;
        waitForStart();



        while (opModeIsActive()) {
            int current_pos = motor.getCurrentPosition();
            if (current_pos >= Constants.LEVER_TICK_COUNT) {
                down = true;
            }
            else if (current_pos <= 0){
                down = false;
            }
            if (gamepad1.square) {
                if (down) {
                    target_pos -= 10;
                }
                else if (!down) {
                    target_pos += 10;
                }
                motor.setTargetPosition(target_pos);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(0.5);
                sleep(100);
            }
            telemetry.addData("Direction: ", down);
            telemetry.addData("Position", current_pos);
            telemetry.update();
        }
    }


}
