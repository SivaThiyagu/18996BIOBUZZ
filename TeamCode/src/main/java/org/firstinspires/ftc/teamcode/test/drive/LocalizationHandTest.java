package org.firstinspires.ftc.teamcode.test.drive;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.automations.pedroPathing.PedroConstants;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

@TeleOp(name = "Pedro Localization Hand Test", group = "Tuning")
public class LocalizationHandTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        String[] names = {HardwareMapNames.DRIVE_FRONT_LEFT, HardwareMapNames.DRIVE_FRONT_RIGHT,
                HardwareMapNames.DRIVE_BACK_LEFT, HardwareMapNames.DRIVE_BACK_RIGHT};
        DcMotor[] motors = new DcMotor[names.length];
        for (int i = 0; i < names.length; i++) {
            motors[i] = hardwareMap.get(DcMotor.class, names[i]);
            motors[i].setPower(0);
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        try {
            GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,
                    HardwareMapNames.PINPOINT);
            PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, PedroConstants.pinpoint);
            pinpoint.resetPosAndIMU();
            while (opModeInInit()) {
                localizer.update();
                telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
                telemetry.addLine("Keep still. Press START when Pinpoint says READY.");
                telemetry.addLine("Forward: +X. Left: +Y. Counterclockwise: heading increases.");
                telemetry.update();
                idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            localizer.update();
            if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                while (opModeIsActive()) {
                    telemetry.addData("Pinpoint status at START", pinpoint.getDeviceStatus());
                    telemetry.addLine("Stop and reinitialize before pushing.");
                    telemetry.update();
                    idle();
                }
                return;
            }
            localizer.setPose(new Pose(0, 0, 0));
            while (opModeIsActive()) {
                localizer.update();
                Pose pose = localizer.getPose();
                telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
                telemetry.addData("X inches", "%.2f", pose.getX());
                telemetry.addData("Y inches", "%.2f", pose.getY());
                telemetry.addData("Heading degrees", "%.1f", Math.toDegrees(pose.getHeading()));
                telemetry.addData("X encoder", pinpoint.getEncoderX());
                telemetry.addData("Y encoder", pinpoint.getEncoderY());
                telemetry.update();
                idle();
            }
        } finally {
            for (DcMotor motor : motors) {
                motor.setPower(0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }
}
