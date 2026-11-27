package org.firstinspires.ftc.teamcode.test.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

@TeleOp(name = "BIOBUZZ Wheel Test", group = "BIOBUZZ")
public class DriveMotorTest extends LinearOpMode {
    private static final double TEST_POWER = 0.20;

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
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addLine("Support chassis with wheels clear of the floor.");
        telemetry.addLine("Hold ONE button: X=front left, Y=front right, A=back left, B=back right.");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive()) {
                int buttons = 0;
                if (gamepad1.x) buttons++;
                if (gamepad1.y) buttons++;
                if (gamepad1.a) buttons++;
                if (gamepad1.b) buttons++;
                boolean oneButton = buttons == 1;

                frontLeft.setPower(oneButton && gamepad1.x ? TEST_POWER : 0);
                frontRight.setPower(oneButton && gamepad1.y ? TEST_POWER : 0);
                backLeft.setPower(oneButton && gamepad1.a ? TEST_POWER : 0);
                backRight.setPower(oneButton && gamepad1.b ? TEST_POWER : 0);

                telemetry.addLine("Hold ONE button; release to stop. Power: 20%.");
                telemetry.addData("X / Square: front left ticks", frontLeft.getCurrentPosition());
                telemetry.addData("Y / Triangle: front right ticks", frontRight.getCurrentPosition());
                telemetry.addData("A / Cross: back left ticks", backLeft.getCurrentPosition());
                telemetry.addData("B / Circle: back right ticks", backRight.getCurrentPosition());
                if (buttons > 1) telemetry.addLine("Multiple buttons: all wheels stopped.");
                telemetry.update();
                idle();
            }
        } finally {
            for (DcMotor motor : motors) motor.setPower(0);
        }
    }
}
