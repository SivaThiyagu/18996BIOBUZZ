package org.firstinspires.ftc.teamcode.test;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "IntakeTest", group = "Sensor")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        telemetry.addLine("Gamepad 2: right trigger = intake, left trigger = reverse.");
        telemetry.addLine("Release both to stop. Maximum power: 25%");
        telemetry.update();

        try {
            waitForStart();
            while (opModeIsActive()) {
                double intakePower = 0.25 * (gamepad2.right_trigger - gamepad2.left_trigger);
                intake.setCustomIntakeSpeed(intakePower);
                telemetry.addData("Intake power", "%.2f", intakePower);
                telemetry.update();
                idle();
            }
        } finally {
            intake.setIntakeState(IntakeSubsystem.IntakeState.DISABLED);
        }
    }
}
