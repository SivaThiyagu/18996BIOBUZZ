package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.teamcode.tools.HardwareMapNames;
import org.firstinspires.ftc.teamcode.LimelightHelpers;

@TeleOp(name = "HighVsLow", group = "Sensor")
public class HighVsLow extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, HardwareMapNames.LIMELIGHT);
        telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(1);
        limelight.start();

        try {
            waitForStart();

            while (opModeIsActive()) {
                LLResult result = limelight.getLatestResult();



                if (result.isValid()) {
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                    for (LLResultTypes.ColorResult colorTarget : colorResults) {
                        List<List<Double>> corners = colorTarget.getTargetCorners();

                        if (corners != null && !corners.isEmpty()) {
                            double lowest_x = 0;
                            double highest_x = 0;
                            double highest_y = -9999999;
                            double lowest_y = 99999999;

                            for (int i = 0; i < corners.size(); i++) {
                                double x = corners.get(i).get(0);
                                double y = corners.get(i).get(1);
                                if (y < lowest_y) {
                                    lowest_x = x;
                                    lowest_y = y;
                                }
                                else if (y > highest_y) {
                                    highest_x = x;
                                    highest_y = y;
                                }
                                telemetry.addData("Corner " + i, "X: %.1f, Y: %.1f", x, y);

                            }
                            if (highest_x > lowest_x) {
                                telemetry.addData("Tipped: ", "right");
                            }
                            else if (highest_x < lowest_x) {
                                telemetry.addData("Tipped: ", "left");
                            }
                            else {
                                telemetry.addData("Tipped: ", "unknown");
                            }

                        }

                       else {
                            telemetry.addLine("Error: Limelight is returning fewer than 4 corners.");
                        }
                        telemetry.update();
                    }
                }
                else {
                    telemetry.addLine("No target visible on screen.");
                }

                telemetry.update();
            }
        }
        finally {
            limelight.stop();
        }
    }
}
