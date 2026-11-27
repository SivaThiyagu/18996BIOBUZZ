
package org.firstinspires.ftc.teamcode.teleops;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ManualShooter;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;


@TeleOp(name = "BIOBUZZ Drive + Intake", group = "BIOBUZZ")
public class DriveAndIntakeTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        TransferSubsystem transfer = new TransferSubsystem(hardwareMap);
        ManualShooter shooter = new ManualShooter(hardwareMap);
        GoBildaPinpointDriver pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, HardwareMapNames.PINPOINT);
        try {
            if (pinpoint != null) pinpoint.recalibrateIMU();
            while (opModeInInit()) {
                if (pinpoint != null) {
                    pinpoint.update();
                    telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
                    telemetry.addLine("Face forward. Hold still until READY.");
                } else {
                    telemetry.addLine("Pinpoint missing | Robot-centric");
                }
                telemetry.addData("P1 D-pad", "Up = set forward | Down = robot");
                telemetry.addData("P2", "Triggers intake | Bumpers paddle | Hold Y spin");
                telemetry.addLine("Paddle: no auto limits");
                telemetry.addData("Shooter", shooter.getStatus(false));
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            double forwardReference = readHeading(pinpoint);
            boolean fieldCentric = pinpoint != null;
            boolean shooterButtonReleased = false;

            while (opModeIsActive()) {
                double heading = readHeading(pinpoint);
                if (!Double.isFinite(heading)) forwardReference = Double.NaN;
                if (gamepad1.dpadUpWasPressed() && Double.isFinite(heading)) {
                    forwardReference = heading;
                    fieldCentric = true;
                }
                if (gamepad1.dpad_down) fieldCentric = false;

                double forward = -gamepad1.left_stick_y;
                double right = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;
                boolean slow = gamepad1.left_bumper;
                String speed = slow ? "Slow 35%" : "Normal";
                if (!fieldCentric) {
                    drive.drive(forward, right, turn, slow);
                    telemetry.addData("Drive", "Robot | %s", speed);
                } else if (Double.isFinite(forwardReference)) {
                    double relativeHeading = heading - forwardReference;
                    drive.driveFieldCentric(forward, right, turn, relativeHeading, slow);
                    telemetry.addData("Drive", "Field | %.0f° | %s",
                            AngleUnit.normalizeDegrees(Math.toDegrees(relativeHeading)), speed);
                } else {
                    drive.stop();
                    telemetry.addData("Drive", "Paused | Down = robot");
                    telemetry.addLine("When READY: face forward, tap Up.");
                }
                if (pinpoint == null || !Double.isFinite(heading)) {
                    telemetry.addData("Pinpoint", pinpoint == null ? "Missing" : pinpoint.getDeviceStatus());
                }
                double intakePower =
                        0.25 * (gamepad2.right_trigger - gamepad2.left_trigger);

                intake.setCustomIntakeSpeed(intakePower);
                if (!gamepad2.y) {
                    shooterButtonReleased = true;
                }
                if (gamepad1.xWasPressed()) {
                    transfer.setPower(0.75);
                }
                boolean spinShooter = shooterButtonReleased && gamepad2.y;
                shooter.setRunning(spinShooter);
                transfer.move(gamepad2.right_bumper, gamepad2.left_bumper);
                if (transfer.getPosition() > tickcount) {
                    transfer.setPower()
                }
                telemetry.addData("Intake", "%.0f%%", intakePower);
                telemetry.addData("Paddle", "%.0f%% | %d ticks", transfer.getPower(), transfer.getPosition());
                telemetry.addData("Shooter", shooter.getStatus(spinShooter));
                telemetry.update();
                idle();
            }
        } finally {
            shooter.stop();
            drive.stop();
            transfer.stop();
            intake.setIntakeState(IntakeSubsystem.IntakeState.DISABLED);
        }
    }

    private double readHeading(GoBildaPinpointDriver pinpoint) {
        if (pinpoint == null) return Double.NaN;
        pinpoint.update();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) return Double.NaN;
        return pinpoint.getHeading(AngleUnit.RADIANS);
    }
}
