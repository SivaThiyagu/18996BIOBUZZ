package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import java.lang.annotation.Target;

@TeleOp(name = "Limelight Snapshot & Analysis", group = "Linear opMode")
public class PictureAnalysis extends LinearOpMode {
     private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
      limelight = hardwareMap.get(Limelight3A.class,"limeLight");
      telemetry.setMsTransmissionInterval(11);
      limelight.pipelineSwitch(0);
      limelight.start();

      telemetry.addData("Status", "Initialized. Looking for Blue!!");
      telemetry.update();

      waitForStart();

      while (opModeIsActive()){
          LLResult result = limelight.getLatestResult();

          if(result != null && result.isValid()){
              double tx = result.getTx();
              double ta = result.getTa();

              telemetry.addData("Blue", "DETECTED!");
              telemetry.addData("Horizontal offset (tx)", tx);
              telemetry.addData("Target Area", ta);

              if (tx > 2.0){
                  telemetry.addLine("Action: Turn Right to center the blue object ");

          }
              else if (tx < -2.0 ){
                  telemetry.addLine("Action: Turn Left to center the blue object ");

              } else{
                  telemetry.addLine("Action: Aligned perfectly with Blue! ");
              }
          } else {
              telemetry.addLine( "NOT FOUND!");
          }
      }
      telemetry.update();
    }


     /*
     @Override
    public void runOpMode() throws InterruptedException {
     limelight = hardwareMap.get(Limelight3A.class,"limelight");
     telemetry.setMsTransmissionInterval(11);
     limelight.pipelineSwitch(0);
     limelight.start();

     telemetry.addData("Status", "Ready!");
     telemetry.update();
     waitForStart();
     while(opModeIsActive()){
         LLResult result = limelight.getLatestResult();
         if (result != null && result.isValid() ){
             double tx = result.getTx();
             double ty = result.getTy();
             double ta = result.getTa();

         } else {
             telemetry.addData("Target Found", "No");
         }
         if (gamepad1.a){
             limelight. captureSnapshot("pit_debug_limage");
             sleep(250);

         }

     }
        telemetry.update();

    }*/
}
