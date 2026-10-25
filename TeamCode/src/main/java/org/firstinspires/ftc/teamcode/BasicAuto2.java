package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.security.cert.PKIXRevocationChecker;

@TeleOp
public class BasicAuto2 extends LinearOpMode {

    private DcMotor motor;
    private Servo servo0;
    private Servo servo1;

    private RevColorSensorV3 sensor;
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        // Mapping first servo
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo0 = hardwareMap.get(Servo.class, "servo0");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        sensor = hardwareMap.get(RevColorSensorV3.class, "sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");





        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                motor.setPower(gamepad1.left_trigger);
            }
            else if  (gamepad1.right_bumper){
                motor.setPower(gamepad1.left_stick_x);
            }
            else {
                motor.setPower(0);
            }

            if (gamepad1.square) {
                servo0.setPosition(0.5);
            }
            else {
                servo0.setPosition(0);
            }
            if (gamepad1.right_trigger_pressed) {
                servo1.setPosition(gamepad1.right_trigger);
            }
            else{
                servo1.setPosition(0);
            }

            float modified_red = (float) sensor.red() / sensor.alpha();
            float modified_green = (float) sensor.green() / sensor.alpha();
            float modified_blue = (float) sensor.blue() / sensor.alpha();
            if (distanceSensor.getDistance(DistanceUnit.CM) < 4 && modified_red > 0.9 && modified_red < 1.05 && modified_blue > 0.4 && modified_blue < 0.6 && modified_green > 1.3 && modified_green < 1.6) {
                telemetry.addData("Pollen? ", "yes");
                servo1.setPosition(1);
            }
            else if (distanceSensor.getDistance(DistanceUnit.CM) > 4){
                telemetry.addData("Pollen? ", "Too Far to tell");
                servo1.setPosition(0);

            }
            else {
                telemetry.addData("Pollen? ", "No");
            }

            telemetry.addData("Distance(CM): ", distanceSensor.getDistance(DistanceUnit.CM));

            telemetry.addData("Red: ", sensor.red());
            telemetry.addData("Blue: ", sensor.blue());
            telemetry.addData("Green: ", sensor.green());
            telemetry.addData("Alpha: ", sensor.alpha());
            telemetry.addData("Modified Red: ", modified_red);
            telemetry.addData("Modified Blue: ", modified_blue);
            telemetry.addData("Modified Green: ", modified_green);

            telemetry.update();
        }
    }
}
