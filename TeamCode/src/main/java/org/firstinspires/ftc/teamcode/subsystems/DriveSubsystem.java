package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

public class DriveSubsystem extends SubsystemBase {
    // Constructor: find motors and set their directions/modes.
    private final DcMotor frontLeftMotor;
    private final DcMotor backLeftMotor;
    private final DcMotor frontRightMotor;
    private final DcMotor backRightMotor;
    public DriveSubsystem(HardwareMap hardwareMap){


            frontLeftMotor = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_FRONT_LEFT);
            backLeftMotor = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_BACK_LEFT);
            frontRightMotor = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_FRONT_RIGHT);
            backRightMotor = hardwareMap.get(DcMotor.class, HardwareMapNames.DRIVE_BACK_RIGHT);


            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);

            // Establish the drive mode each time this subsystem is created.
            DcMotor[] motors = {frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor};
            for (DcMotor motor : motors) {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
    }

    // Method: calculate and apply the four wheel powers.
    public void drive(double forward, double strafe, double turn, boolean slow) {
        double max;


        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;


        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }


        double driveScale = slow ? 0.35 : 1.0;
        frontLeftPower *= driveScale;
        frontRightPower *= driveScale;
        backLeftPower *= driveScale;
        backRightPower *= driveScale;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void driveFieldCentric(double fieldForward, double fieldRight, double turn,
                                 double headingRadians, boolean slow) {
        double cos = Math.cos(headingRadians);
        double sin = Math.sin(headingRadians);
        double robotForward = fieldForward * cos - fieldRight * sin;
        double robotRight = fieldForward * sin + fieldRight * cos;
        drive(robotForward, robotRight, turn, slow);
    }

    // Method: set all four motor powers to zero.
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);


    }

}
