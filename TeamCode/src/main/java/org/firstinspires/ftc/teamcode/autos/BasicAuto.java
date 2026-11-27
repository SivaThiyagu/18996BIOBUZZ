package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name = "Basic BioBuzz Auto")
public class BasicAuto extends LinearOpMode {

    private static final double DRIVE_POWER = 0.5;
    private static final double DRIVE_SECONDS = 1.0;

    @Override
    public void runOpMode() {

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        telemetry.addData("Timed forward drive (seconds)", DRIVE_SECONDS);
        telemetry.addData("Drive power", DRIVE_POWER);
        telemetry.update();
        try {
            waitForStart();
            ElapsedTime timer = new ElapsedTime();
            while (opModeIsActive() && timer.seconds() < DRIVE_SECONDS) {
                drive.drive(DRIVE_POWER, 0, 0, false);
                idle();
            }
        } finally {
            drive.stop();
        }
    }
}
