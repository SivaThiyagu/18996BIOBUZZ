package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

@Autonomous(name = "test", group = "tests")
public class BasicTest extends LinearOpMode {
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_FRONT_LEFT);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_FRONT_RIGHT);
        DcMotor backLeft = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_BACK_LEFT);
        DcMotor backRight = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_BACK_RIGHT);
        for (DcMotor motor : new DcMotor[]{frontLeft, frontRight, backLeft, backRight}) {
            motor.setPower(0);
        }
        waitForStart();
        while (opModeIsActive()) {

            // Force BRAKE mode if not already set by Pedro:
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Print to Driver Station telemetry to verify:
            telemetry.addData("FL Mode", frontLeft.getZeroPowerBehavior());
            telemetry.addData("FR Mode", frontRight.getZeroPowerBehavior());
            telemetry.addData("BL Mode", backLeft.getZeroPowerBehavior());
            telemetry.addData("BR Mode", backRight.getZeroPowerBehavior());
            telemetry.update();
        }
    }
}
