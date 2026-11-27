package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.automations.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ManualShooter;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;
//hypothetical
@Autonomous(name = "BIOBUZZ Score + Park", group = "BIOBUZZ")
public class RealAuto extends LinearOpMode {
    private static final Pose START = waypoint(0, 0, 0);
    private static final Pose LEAVE = null;
    private static final Pose PICKUP = null;
    private static final Pose SHOOT = null;
    private static final Pose PARK = null;

    private static final boolean ENABLE_SCORING = false;
    private static final double SPIN_UP_SECONDS = 0;
    private static final double FEED_FORWARD_SECONDS = 0;
    private static final double FEED_REVERSE_SECONDS = 0;
    private static final int FEED_CYCLES = 1;

    private static final double DRIVE_POWER = 0.25;
    private static final double PICKUP_POWER = 0.25;
    private static final double LEG_TIMEOUT_SECONDS = 5;
    private static final double AUTO_LIMIT_SECONDS = 28;
    private static final double POSITION_TOLERANCE_INCHES = 3;
    private static final double HEADING_TOLERANCE_DEGREES = 5;

    private final ElapsedTime matchTimer = new ElapsedTime();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {
        String routeProblem = routeProblem();
        if (routeProblem != null) {
            showResult(routeProblem);
            waitForStart();
            return;
        }

        ManualShooter shooter = ENABLE_SCORING ? new ManualShooter(hardwareMap) : null;
        if (shooter != null && !shooter.isReady()) {
            showResult(shooter.getStatus(false));
            waitForStart();
            return;
        }

        IntakeSubsystem intake = PICKUP != null ? new IntakeSubsystem(hardwareMap) : null;
        TransferSubsystem transfer = ENABLE_SCORING ? new TransferSubsystem(hardwareMap) : null;
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwareMapNames.PINPOINT);
        follower = PedroConstants.createFollower(hardwareMap);
        try {
            pinpoint.resetPosAndIMU();
            while (opModeInInit()) {
                pinpoint.update();
                telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
                telemetry.addData("Scoring", ENABLE_SCORING ? "Enabled" : "Disabled");
                telemetry.addData("Pickup", PICKUP == null ? "Skipped" : "Enabled");
                telemetry.addLine("Keep still. Press START when Pinpoint says READY.");
                telemetry.update();
                idle();
            }
            waitForStart();
            matchTimer.reset();
            if (!opModeIsActive()) return;
            pinpoint.update();
            if (!canContinue()) {
                showResult("AUTO aborted before drive");
                return;
            }
            follower.setStartingPose(START);
            follower.setMaxPower(DRIVE_POWER);

            Pose nextLegStart = START;
            if (LEAVE != null) {
                if (!driveLeg("Leave", nextLegStart, LEAVE)) return;
                nextLegStart = LEAVE;
            }
            if (intake != null) {
                intake.setCustomIntakeSpeed(PICKUP_POWER);
                try {
                    if (!driveLeg("Pickup", nextLegStart, PICKUP)) return;
                } finally {
                    intake.setCustomIntakeSpeed(0);
                }
                nextLegStart = PICKUP;
            }

            if (SHOOT != null) {
                if (!driveLeg("Drive to shooting position", nextLegStart, SHOOT)) return;
                nextLegStart = SHOOT;
            }
            if (shooter != null && !shoot(shooter, transfer)) return;
            if (!driveLeg("Park", nextLegStart, PARK)) return;
            showResult("Sequence complete. Check actual park position.");
        } finally {
            follower.breakFollowing();
            if (shooter != null) shooter.stop();
            if (transfer != null) transfer.stop();
            if (intake != null) intake.setCustomIntakeSpeed(0);
        }
    }

    private boolean driveLeg(String name, Pose start, Pose target) {
        if (!canContinue()) {
            showResult(name + " aborted before drive");
            return false;
        }
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(start, target))
                .setLinearHeadingInterpolation(start.getHeading(), target.getHeading())
                .build();
        ElapsedTime stepTimer = new ElapsedTime();
        follower.followPath(path, false);
        try {
            while (canContinue() && follower.isBusy() && stepTimer.seconds() < LEG_TIMEOUT_SECONDS) {
                update(name);
            }
        } finally {
            follower.breakFollowing();
        }

        Pose actual = follower.getPose();
        double headingError = target.getHeading() - actual.getHeading();
        double wrappedError = Math.atan2(Math.sin(headingError), Math.cos(headingError));
        boolean atTarget = distance(actual, target) <= POSITION_TOLERANCE_INCHES
                && Math.abs(Math.toDegrees(wrappedError)) <= HEADING_TOLERANCE_DEGREES;
        if (!canContinue() || stepTimer.seconds() >= LEG_TIMEOUT_SECONDS || !atTarget) {
            showResult(name + " aborted: target missed, timeout, STOP, or Pinpoint fault");
            return false;
        }
        return true;
    }

    private boolean shoot(ManualShooter shooter, TransferSubsystem transfer) {
        if (!canContinue()) {
            showResult("Shooting aborted before spin-up");
            return false;
        }
        shooter.setRunning(true);
        try {
            if (!runTimedStep("Shooter spin-up", SPIN_UP_SECONDS)) return false;
            for (int i = 0; i < FEED_CYCLES; i++) {
                if (!canContinue()) return false;
                transfer.move(true, false);
                if (!runTimedStep("Feed forward", FEED_FORWARD_SECONDS)) return false;
                if (!canContinue()) return false;
                transfer.move(false, true);
                if (!runTimedStep("Return paddle", FEED_REVERSE_SECONDS)) return false;
                transfer.stop();
            }
            return true;
        } finally {
            transfer.stop();
            shooter.stop();
        }
    }

    private boolean runTimedStep(String name, double seconds) {
        ElapsedTime stepTimer = new ElapsedTime();
        while (canContinue() && stepTimer.seconds() < seconds) {
            update(name);
        }
        if (!canContinue()) {
            showResult(name + " aborted");
            return false;
        }
        return true;
    }

    private void update(String step) {
        follower.update();
        Pose pose = follower.getPose();
        telemetry.addData("Step", step);
        telemetry.addData("AUTO seconds", "%.1f", matchTimer.seconds());
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addData("Pose (X, Y, degrees)", "%.1f, %.1f, %.1f",
                pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.update();
        idle();
    }

    private boolean canContinue() {
        return opModeIsActive() && matchTimer.seconds() < AUTO_LIMIT_SECONDS
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    private void showResult(String result) {
        telemetry.addData("Result", result);
        if (pinpoint != null) telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        if (follower != null) {
            Pose pose = follower.getPose();
            telemetry.addData("Final pose (X, Y, degrees)", "%.1f, %.1f, %.1f",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        }
        telemetry.update();
    }

    private static String routeProblem() {
        if (!PedroConstants.ROBOT_SETTINGS_VERIFIED) return "Verify Pedro localization first";
        if (PARK == null) return "Set the measured PARK waypoint";
        if (ENABLE_SCORING && SHOOT == null) return "Set the measured SHOOT waypoint for scoring";

        int legCount = 0;
        Pose previous = START;
        for (Pose target : new Pose[]{LEAVE, PICKUP, SHOOT, PARK}) {
            if (target == null) continue;
            double legDistance = distance(previous, target);
            if (!(legDistance > 0) || !Double.isFinite(legDistance)
                    || !Double.isFinite(target.getHeading())) {
                return "Consecutive waypoints need different, finite positions";
            }
            previous = target;
            legCount++;
        }

        if (ENABLE_SCORING) {
            if (!(SPIN_UP_SECONDS > 0 && FEED_FORWARD_SECONDS > 0 && FEED_REVERSE_SECONDS > 0)
                    || FEED_CYCLES < 1 || FEED_CYCLES > 4) {
                return "Set measured spin-up, paddle up and return times, and 1 to 4 cycles";
            }
            double scoringSeconds = SPIN_UP_SECONDS
                    + FEED_CYCLES * (FEED_FORWARD_SECONDS + FEED_REVERSE_SECONDS);
            if (legCount * LEG_TIMEOUT_SECONDS + scoringSeconds >= AUTO_LIMIT_SECONDS) {
                return "Route and scoring time limits exceed the AUTO budget";
            }
        }
        return null;
    }

    private static double distance(Pose from, Pose to) {
        return Math.hypot(to.getX() - from.getX(), to.getY() - from.getY());
    }

    private static Pose waypoint(double xInches, double yInches, double headingDegrees) {
        return new Pose(xInches, yInches, Math.toRadians(headingDegrees));
    }
}
