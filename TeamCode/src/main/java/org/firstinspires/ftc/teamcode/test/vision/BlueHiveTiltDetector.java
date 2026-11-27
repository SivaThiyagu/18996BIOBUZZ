package org.firstinspires.ftc.teamcode.test.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.List;

@TeleOp(name = "Blue Hive Tilt Detector")
public class BlueHiveTiltDetector extends LinearOpMode {

    private Limelight3A limelight;

    private static final double TOLERANCE = 5.0;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        limelight.start();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            String tilt = "UNKNOWN";
            double angle = 0;

            if (result != null && result.isValid()) {

                List<LLResultTypes.ColorResult> targets =
                        result.getColorResults();

                if (targets != null && !targets.isEmpty()) {

                    LLResultTypes.ColorResult target =
                            targets.get(0);

                    // Corner coordinates from Limelight
                    List<List<Double>> corners =
                            target.getTargetCorners();

                    if (corners != null && corners.size() >= 4) {

                        // Calculate orientation using
                        // the first two corners.
                        telemetry.addData("Tx: ", result.getTx());
                        telemetry.addData("Ty: ", result.getTy());
                        telemetry.addData("Ta: ", result.getTa());

                        double x1 = corners.get(0).get(0);
                        double y1 = corners.get(0).get(1);

                        double x2 = corners.get(1).get(0);
                        double y2 = corners.get(1).get(1);

                        // Ensure left-to-right ordering
                        if (x1 > x2) {
                            double tempX = x1;
                            x1 = x2;
                            x2 = tempX;

                            double tempY = y1;
                            y1 = y2;
                            y2 = tempY;
                        }

                        angle = Math.toDegrees(
                                Math.atan2(y2 - y1, x2 - x1)
                        );

                        if (Math.abs(angle) <= TOLERANCE) {
                            tilt = "LEVEL";
                        } else if (angle > 0) {
                            tilt = "LEFT";
                        } else {
                            tilt = "RIGHT";
                        }

                    } else {
                        tilt = "NO CORNERS";
                    }

                } else {
                    tilt = "NO BLUE TARGET";
                }

            } else {
                tilt = "NO VALID RESULT";
            }

            telemetry.addData("Hive Tilt", tilt);
            telemetry.addData("Angle", "%.2f", angle);
            telemetry.update();

            sleep(50);
        }

        limelight.stop();
    }
}