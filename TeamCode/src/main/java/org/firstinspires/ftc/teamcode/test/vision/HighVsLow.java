package org.firstinspires.ftc.teamcode.test.vision;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

@TeleOp(name = "HighVsLow", group = "Sensor")
public class HighVsLow extends LinearOpMode {
    @Override
    public void runOpMode() {
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, HardwareMapNames.LIMELIGHT);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        try {
            waitForStart();
            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();
                if (result == null || !result.isValid()) {
                    telemetry.addLine("No target visible");
                } else if (result.getPipelineIndex() != 0 || result.getStaleness() > 200) {
                    telemetry.addLine("Need better data from pipeline 0");
                } else {
                    List<LLResultTypes.ColorResult> targets = result.getColorResults();
                    if (targets == null || targets.size() != 1) {
                        telemetry.addLine("must be 1 color target");
                    } else {
                        List<List<Double>> corners = targets.get(0).getTargetCorners();
                        boolean validCorners = (corners != null && corners.size() == 4);
                        if (validCorners) {
                            for (List<Double> corner : corners) {
                                if (corner == null || corner.size() < 2
                                        || corner.get(0) == null || corner.get(1) == null) {
                                    validCorners = false;
                                    break;
                                }
                            }
                        }

                        if (!validCorners) {
                            telemetry.addLine("Need four corners with X and Y coordinates");
                        } else {
                            double topX = corners.get(0).get(0);
                            double topY = corners.get(0).get(1);
                            double bottomX = topX;
                            double bottomY = topY;

                            for (List<Double> corner : corners) {
                                double x = corner.get(0);
                                double y = corner.get(1);
                                if (y < topY) {
                                    topX = x;
                                    topY = y;
                                }
                                if (y > bottomY) {
                                    bottomX = x;
                                    bottomY = y;
                                }
                            }

                            telemetry.addData("Top corner X / Y", "%.1f / %.1f", topX, topY);
                            telemetry.addData("Bottom corner X / Y", "%.1f / %.1f", bottomX, bottomY);
                            if (topX > bottomX) {
                                telemetry.addData("Tilt: ", "Top is right of bottom, titled left");
                            } else if (topX < bottomX) {
                                telemetry.addData("Tilt: ", "Top is left of bottom, titled right");
                            } else {
                                telemetry.addData("Tilt: ", "Same X position, balanced");
                            }
                        }
                    }
                }
                telemetry.update();
                sleep(100);
            }
        } finally {
            limelight.stop();
        }
    }
}
