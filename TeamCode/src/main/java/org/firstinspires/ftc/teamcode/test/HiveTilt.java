package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@TeleOp
public class HiveTilt extends LinearOpMode {
    private Limelight3A limelight;

    private static final int BlUE_PIPELINE = 0;
    private static  final int RED_PIPELINE = 1;

    private final boolean isRedAlliance = true;



    @Override
    public void runOpMode(){
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        int activePipeline =  isRedAlliance ? RED_PIPELINE : BlUE_PIPELINE;
        limelight.pipelineSwitch(activePipeline);

        telemetry.addData("Status","Initialized");
        telemetry.addData("Pipeline", isRedAlliance ? "1(Red)": "0(Blue)");
        telemetry.update();

        waitForStart();

        String tiltedSide = detectTiltedSide(activePipeline,1.0);

        telemetry.addData("Detected Tilted Side",tiltedSide);
        telemetry.update();

        if (tiltedSide.equals("LEFT")) {
            scoreLeftCell();
        } else if (tiltedSide.equals("RIGHT")) {
            scoreRightCell();
        } else {
            fallbackPark();

        }

         while (opModeIsActive()) {
             idle();
         }


    }

    private String detectTiltedSide (int targetPipeline, double timeoutSeconds){
        ElapsedTime timer  = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < timeoutSeconds){
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()
            && result.getPipelineIndex() == targetPipeline
            && result.getStaleness() < 150){
              List<LLResultTypes.ColorResult> colorTargets = result.getColorResults();

              for(LLResultTypes.ColorResult target : colorTargets){
                  double area = target.getTargetArea();
                  double tx = target.getTargetXDegrees();
                  double ty = target.getTargetYDegrees();

                  if(area > 1.0 && ty > 0 ){
                      return (tx < 0) ? "LEFT" : "RIGHT";
                  }
              }
              sleep(20);
            }

        }

        return  "None";
    }

    private void scoreLeftCell() {
        // Left cell motor and servo code goes here
    }

    private void scoreRightCell() {
        // Right cell motor and servo code goes here
    }

    private void fallbackPark() {
        // Backup parking motor code goes here
    }




}
