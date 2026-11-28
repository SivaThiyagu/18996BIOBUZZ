package org.firstinspires.ftc.teamcode.automations.pedroPathing;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Paths", group = "Autonomous")
public class Paths extends LinearOpMode {


    private Follower follower;
    private PathChain route;

    // Placeholder coordinates in inches: replace with your field positions.
    // Headings must be in radians.
    private final Pose startPose = new Pose(56, 8, Math.toRadians(180));
    private final Pose poseOne = new Pose(56, 36, Math.toRadians(180));
    private final Pose poseTwo = new Pose(30, 60, Math.toRadians(90));

    @Override
    public void runOpMode() {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose); // robot must be physically placed here
        buildPaths();

        // Replaces waitForStart() so localization keeps updating during init
        while (opModeInInit()) {
            follower.update();
            showTelemetry();
        }

        if (isStopRequested()) return;

        follower.followPath(route, true); // true = hold pose3 at the end

        // Keep looping until Stop so the follower can drive and then hold pose3
        while (opModeIsActive()) {
            follower.update(); // must be called every loop
            showTelemetry();
        }
    }

    private void buildPaths() {
        route = follower.pathBuilder()
                // startPose -> poseOne: hold 180 degrees
                .addPath(new BezierLine(startPose, poseOne))
                .setConstantHeadingInterpolation(poseOne.getHeading())
                // poseOne -> poseTwo: turn from 180 to 90 degrees while driving
                .addPath(new BezierLine(poseOne, poseTwo))
                .setLinearHeadingInterpolation(poseTwo.getHeading(), poseTwo.getHeading())
                .build();
    }

    private void showTelemetry() {
        Pose current = follower.getPose();
        telemetry.addData("Following", follower.isBusy());
        telemetry.addData("X", current.getX());
        telemetry.addData("Y", current.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(current.getHeading()));
        telemetry.update();
    }

}
