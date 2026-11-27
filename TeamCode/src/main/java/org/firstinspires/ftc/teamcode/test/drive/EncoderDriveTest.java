package org.firstinspires.ftc.teamcode.test.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

@Autonomous(name = "BIOBUZZ Encoder", group = "BIOBUZZ")
public class EncoderDriveTest extends LinearOpMode {
    private static final int TEST_TICKS = 250;
    private static final double DRIVE_POWER = 0.25;
    private static final double TIMEOUT_SECONDS = 3.0;

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_FRONT_LEFT);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_FRONT_RIGHT);
        DcMotor backLeft = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_BACK_LEFT);
        DcMotor backRight = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_BACK_RIGHT);
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight};


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        telemetry.addData("Status", "Ready for a short forward test");
        telemetry.addData("Target per wheel (ticks)", TEST_TICKS);
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        int[] startTicks = new int[motors.length];
        boolean reachedTargets = false;
        try {
            for (int i = 0; i < motors.length; i++) {
                startTicks[i] = motors[i].getCurrentPosition();
                motors[i].setTargetPosition(startTicks[i] + TEST_TICKS);
                motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            ElapsedTime timer = new ElapsedTime();
            for (DcMotor motor : motors) {
                motor.setPower(DRIVE_POWER);
            }

            while (opModeIsActive() && timer.seconds() < TIMEOUT_SECONDS) {
                reachedTargets = !frontLeft.isBusy() && !frontRight.isBusy()
                        && !backLeft.isBusy() && !backRight.isBusy();
                if (reachedTargets) {
                    break;
                }
                showTravel(motors, startTicks);
                telemetry.update();
            }
        } finally {
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }
            for (DcMotor motor : motors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }

        if (opModeIsActive()) {
            telemetry.addData("Status", reachedTargets ? "Targets reached" : "Timed out");
            showTravel(motors, startTicks);
            telemetry.update();
            sleep(1000);
        }
    }

    private void showTravel(DcMotor[] motors, int[] startTicks) {
        telemetry.addData("Target per wheel (ticks)", TEST_TICKS);
        telemetry.addData("Front left / right (ticks)", "%d / %d",
                motors[0].getCurrentPosition() - startTicks[0],
                motors[1].getCurrentPosition() - startTicks[1]);
        telemetry.addData("Back left / right (ticks)", "%d / %d",
                motors[2].getCurrentPosition() - startTicks[2],
                motors[3].getCurrentPosition() - startTicks[3]);
    }
}
