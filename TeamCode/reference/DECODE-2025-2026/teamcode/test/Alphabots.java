package org.firstinspires.ftc.teamcode.test;

import static java.lang.Math.PI;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.automations.pedroPathing.Constants;

@Disabled
@TeleOp
public class Alphabots extends LinearOpMode {

    Follower follower;
    boolean headingLock;
    PIDFController headingController;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        headingController = new PIDFController(follower.constants.coefficientsHeadingPIDF);

        while (opModeIsActive()) {
           if (gamepad1.rightBumperWasPressed()) {
               headingLock = true;
           } else if (gamepad1.rightBumperWasReleased()) {
               headingLock = false;
           }

           setTeleOpVectors(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

           follower.update();
        }
    }

    public void setTeleOpVectors(double gamepadX, double gamepadY, double gamepadH) {
        double headingError = -PI/2 - follower.getHeading();

        headingController.updateError(headingError);

        double x;
        double y;
        double h;

        x = -gamepadY;
        y = -gamepadX;

        if (headingLock) {
            h = headingController.run();
        } else {
            h = -gamepadH;
        }

        follower.setTeleOpDrive(x, y, h);
    }
}
