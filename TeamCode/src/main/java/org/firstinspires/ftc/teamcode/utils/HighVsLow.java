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
                LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");

                if (results.vpr.isValid && results.vpr.retroOutputs.length > 0) {

                    double[] rawX = results.vpr.retroOutputs.tcornx;
                    double[] rawY = results.vpr.retroOutputs.tcorny;

                    if (rawX != null && rawY != null && rawX.length >= 4) {

                        // Corner 1 (Index 0)
                        double c1_x = rawX[0];
                        double c1_y = rawY[0];

                        // Corner 2 (Index 1)
                        double c2_x = rawX[1];
                        double c2_y = rawY[1];

                        // Corner 3 (Index 2)
                        double c3_x = rawX[2];
                        double c3_y = rawY[2];

                        // Corner 4 (Index 3)
                        double c4_x = rawX[3];
                        double c4_y = rawY[3];


                    } else {
                        telemetry.addLine("Error: Limelight is returning fewer than 4 corners.");
                    }
                else{
                        telemetry.addLine("No target visible on screen.");
                    }
        finally {
            limelight.stop();
                }
            }

            }
