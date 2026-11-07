package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import org.firstinspires.ftc.teamcode.tools.HardwareMapNames;

@TeleOp(name = "HighVsLow", group = "Sensor")
public class HighVsLow extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, HardwareMapNames.LIMELIGHT);
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        try {
            waitForStart();

            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid() && result.getStaleness() <= 100) {
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                    List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

                    for (int i = 0; i < colorResults.size(); i++) {
                        LLResultTypes.ColorResult cr = colorResults.get(i);

                        telemetry.addData("Target #" + i, "X: %.2f, Y: %.2f, Area: %.2f%%",
                                cr.getTargetXDegrees(),
                                cr.getTargetYDegrees(),
                                cr.getTargetArea());
                        telemetry.addData("Color Result: ", cr);

                    }
                    telemetry.addData("Detections Result: ", detections);
                }
                else {
                    if (result == null) {
                        telemetry.addData("Status", "Error: Result object is NULL. Check camera connection/initialization.");
                    } else if (result.getStaleness() > 100) {
                        telemetry.addData("Status", "Stale camera data; ignore observation.");
                    } else if (!result.isValid()) {
                        telemetry.addData("Status", "Connected, but result is INVALID. (Camera sees no targets).");
                    }
                }
                telemetry.update();
            }
        } finally {
            limelight.stop();
        }
    }
}
