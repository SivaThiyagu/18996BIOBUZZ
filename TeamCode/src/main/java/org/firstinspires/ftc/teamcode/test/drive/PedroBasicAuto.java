package org.firstinspires.ftc.teamcode.test.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.automations.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;


@Autonomous(name = "BIOBUZZ Basic Auto", group = "Tests")
public class PedroBasicAuto extends LinearOpMode {
    private static final double DISTANCE_INCHES = 12;
    private static final double TIMEOUT_SECONDS = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        if (!PedroConstants.ROBOT_SETTINGS_VERIFIED) {
            telemetry.addLine("Drive disabled. Verify pod type, directions, offsets and hand-test results.");
            telemetry.update();
            waitForStart();
            return;
        }

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                HardwareMapNames.PINPOINT);
        Follower follower = PedroConstants.createFollower(hardwareMap);
        String result;
        double maxSidewaysInches = 0;
        try {
            pinpoint.resetPosAndIMU();
            while (opModeInInit()) {
                pinpoint.update();
                telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
                telemetry.addData("Forward distance in inches", DISTANCE_INCHES);
                telemetry.addLine("Keep still. Press START when Pinpoint says READY.");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (!opModeIsActive()) return;
            pinpoint.update();
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                telemetry.addData("Drive aborted: Pinpoint status", pinpoint.getDeviceStatus());
                telemetry.update();
                return;
            }

            Pose start = new Pose(0, 0, 0);
            follower.setStartingPose(start);
            follower.setMaxPower(0.25);

            PathChain forward = follower.pathBuilder()
                    .addPath(new BezierLine(start, new Pose(DISTANCE_INCHES, 0, 0)))
                    .setConstantHeadingInterpolation(0)
                    .build();

            ElapsedTime timer = new ElapsedTime();
            follower.followPath(forward, false);
            while (opModeIsActive() && follower.isBusy() && timer.seconds() < TIMEOUT_SECONDS) {

                follower.update();
                if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                    break;
                }
                Pose pose = follower.getPose();
                maxSidewaysInches = Math.max(maxSidewaysInches, Math.abs(pose.getY()));
                showPose(pose);
                telemetry.update();
                idle();
            }
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                result = "Pinpoint fault: " + pinpoint.getDeviceStatus();
            } else if (isStopRequested()) {
                result = "Stopped";
            } else if (follower.isBusy()) {
                result = "Timed out";
            } else {
                result = "Path ended";
            }
        } finally {
            follower.breakFollowing();
        }
        while (opModeIsActive()) {
            follower.updatePose();
            telemetry.addData("Result", result);
            telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            telemetry.addData("Target (X, Y, degrees)", "%.1f, 0, 0", DISTANCE_INCHES);
            showPose(follower.getPose());
            telemetry.addData("Largest sideways reading during drive (in)", "%.2f", maxSidewaysInches);
            telemetry.addLine("Wheels stopped. Measure actual travel, then press STOP.");
            telemetry.update();
            idle();
        }
    }

    private void showPose(Pose pose) {
        telemetry.addData("X (inches)", "%.2f", pose.getX());
        telemetry.addData("Y (inches)", "%.2f", pose.getY());
        telemetry.addData("Heading (degrees)", "%.1f", Math.toDegrees(pose.getHeading()));
    }
}
