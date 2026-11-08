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
                LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults("limelight");

                // Corrected outer IF layout
                if (results.vpr.retroOutputs.length > 0) {


                    double[] rawX = results.vpr.retroOutputs[0].tcornx;
                    double[] rawY = results.vpr.retroOutputs[0].tcorny;

                    if (rawX != null && rawY != null && rawX.length >= 4) {
                        // Corner 1
                        double c1_x = rawX[0];
                        double c1_y = rawY[0];

                        // Corner 2
                        double c2_x = rawX[1];
                        double c2_y = rawY[1];

                        // Corner 3
                        double c3_x = rawX[2];
                        double c3_y = rawY[2];

                        // Corner 4
                        double c4_x = rawX[3];
                        double c4_y = rawY[3];
                        telemetry.addData("c1x: ",  c1_x);
                        telemetry.addData("c1y: ",  c1_y);
                        telemetry.addData("c2x: ",  c2_x);
                        telemetry.addData("c2y: ",  c2_y);
                        telemetry.addData("c3x: ",  c3_x);
                        telemetry.addData("c3y: ",  c3_y);
                        telemetry.addData("c4x: ",  c4_x);
                        telemetry.addData("c4y: ",  c4_y);



                        telemetry.update();
                    } else {
                        telemetry.addLine("Error: Limelight is returning fewer than 4 corners.");
                    }
                } else {
                    telemetry.addLine("No target visible on screen.");
                }

                telemetry.update();
            }
        } finally {
            limelight.stop();
        }
    }
}
