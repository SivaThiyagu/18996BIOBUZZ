
package org.firstinspires.ftc.teamcode.test.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.automations.pedroPathing.PedroConstants;

//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Move Forward")
@Disabled
public class MoveForwardAuto extends LinearOpMode {

    private Follower follower;

    private final Pose startPose = new Pose(0, 0, 0);

    @Override
    public void runOpMode() {

        // Initialize Pedro Pathing
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        driveForward(24);

        while (opModeIsActive() && follower.isBusy()) {

            follower.update();

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
    }

    public void driveForward(double distance) {

        Pose currentPose = follower.getPose();

        double heading = currentPose.getHeading();

        Pose targetPose = new Pose(
                currentPose.getX() + distance * Math.cos(heading),
                currentPose.getY() + distance * Math.sin(heading),
                heading
        );

        PathChain forwardPath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, targetPose))
                .setConstantHeadingInterpolation(heading)
                .build();

        follower.followPath(forwardPath);
    }
}
