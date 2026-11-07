package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.tools.HardwareMapNames;
import java.util.List;

/** Reports individual tags only. This diagnostic never commands motors or estimates field pose. */
@TeleOp(name = "BIOBUZZ Limelight AprilTags", group = "Sensor")
public class LimelightAprilTagTest extends LinearOpMode {
    // Configure this index as an AprilTag pipeline in the Limelight interface.
    // Color/detector experiments may need a different pipeline index.
    private static final int APRILTAG_PIPELINE = 0;
    // Provisional threshold; validate with the actual camera and Control Hub.
    private static final long MAX_RESULT_AGE_MS = 100;

    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, HardwareMapNames.LIMELIGHT);
        telemetry.setMsTransmissionInterval(100);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(APRILTAG_PIPELINE);
        limelight.start();
        try {
            telemetry.addData("Status", "Ready. Press Play.");
            telemetry.addData("Setup", "Configure pipeline %d for AprilTags", APRILTAG_PIPELINE);
            telemetry.update();
            waitForStart();
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                boolean usable = false;
                telemetry.addData("Connected", limelight.isConnected());
                if (result == null) {
                    telemetry.addData("Status", "No camera data");
                } else {
                    telemetry.addData("Result age (ms)", result.getStaleness());
                    telemetry.addData("Pipeline", "%d (%s)", result.getPipelineIndex(), result.getPipelineType());
                    if (result.getPipelineIndex() != APRILTAG_PIPELINE) {
                        telemetry.addData("Status", "Waiting for AprilTag pipeline");
                    } else if (result.getStaleness() > MAX_RESULT_AGE_MS) {
                        telemetry.addData("Status", "Stale observation; ignore target");
                    } else if (!result.isValid()) {
                        telemetry.addData("Status", "No valid target");
                    } else {
                        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
                        telemetry.addData("Tag count", tags.size());
                        usable = limelight.isConnected() && !tags.isEmpty();
                        telemetry.addData("Status", usable ? "Fresh AprilTags detected" : "No usable AprilTags");
                        if (usable) {
                            for (int i = 0; i < tags.size(); i++) {
                                LLResultTypes.FiducialResult tag = tags.get(i);
                                telemetry.addData("Tag " + i, "ID %d | horizontal %.2f deg | vertical %.2f deg",
                                        tag.getFiducialId(), tag.getTargetXDegrees(), tag.getTargetYDegrees());
                            }
                        }
                    }
                }
                telemetry.addData("Usable observation", usable);
                telemetry.update();
                idle();
            }
        } finally {
            limelight.stop();
        }
    }
}
