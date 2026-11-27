package org.firstinspires.ftc.teamcode.test;

import static java.lang.Math.PI;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.automations.odo.LimelightManager;
import org.firstinspires.ftc.teamcode.automations.odo.STATICLocalizer;

@Disabled
@TeleOp
public class limelightetst extends LinearOpMode {

    @Override
    public void runOpMode() {

        LimelightManager limelightManager = new LimelightManager(hardwareMap);
        STATICLocalizer staticLocalizer = new STATICLocalizer(hardwareMap, new Pose(0, 0, PI), STATICLocalizer.LocalizerMode.PINPOINT_ONLY);

        waitForStart();

        while (opModeIsActive()) {

            staticLocalizer.update();

            telemetry.addData("MOTIF", GlobalDataStorage.motif);
            telemetry.addData("HEADING", Math.toDegrees(staticLocalizer.getIMUHeading()));

            telemetry.addData("HEA", staticLocalizer.getPose().getHeading());

            telemetry.addData("POSE", limelightManager.getPose(staticLocalizer.getIMUHeading()));
            telemetry.addData("MT2", limelightManager.getPoseMT2());
            telemetry.addData("MT1", limelightManager.getPoseMT1());
            telemetry.update();
        }
    }
}
