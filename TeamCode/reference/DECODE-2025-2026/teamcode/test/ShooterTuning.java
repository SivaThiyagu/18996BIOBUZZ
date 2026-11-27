package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.Robot.Team.BLUE;
import static org.firstinspires.ftc.teamcode.Robot.Team.RED;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.commands.ProgressTransferCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.List;

@TeleOp
public class ShooterTuning extends CommandOpMode {
    private Robot robot;

    private GamepadEx gamepad;

    private double previousVelocityError = 0;

    // Commands are single instanced for teleop
    private Command adjustShooterSpeedCommand = new InstantCommand();
    private Command holdPointCommand = new InstantCommand();
    private Command progressTransferCommand = new InstantCommand();

    @IgnoreConfigurable
    private static TelemetryManager panelsTelemetry;

    public void runOpMode() throws InterruptedException {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));

        waitForStart();
        reset();
        initialize();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);
            run();
        }
    }

    @Override
    public void initialize() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Robot(hardwareMap, GlobalDataStorage.autoTeam, true);

        gamepad = new GamepadEx(gamepad1);

        // FIRE ONE
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(() -> {
                    progressTransferCommand = new ProgressTransferCommand(robot);
                    schedule(progressTransferCommand);
                });

        // FIRE ALL
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    progressTransferCommand = new ProgressTransferCommand(robot, true);
                    schedule(progressTransferCommand);
                });

        // SET SPEED AND ANGLE, CLOSE LAUNCH ZONE (Left Back Paddle)
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(() -> {
                    robot.shooter.setTargetTps(1600);
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);
                });

        // SET SPEED AND ANGLE, CLOSE LAUNCH ZONE (Right Back Paddle)
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(() -> {
                    robot.shooter.setTargetTps(1400);
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);
                });

        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(() -> {
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);
                });

        // SET SPEED AND ANGLE, FAR LAUNCH ZONE
        // todo
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> {
                    if (GlobalDataStorage.team == BLUE) robot.drive.follower.setPose(new Pose(19.9, 120.6, Math.toRadians(141.7)));
                    else if (GlobalDataStorage.team == RED) robot.drive.follower.setPose(new Pose(19.9, 120.6, Math.toRadians(141.7)).mirror());
                });

        // EDIT INSTANCED
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> robot.shooter.setLocalAngle(robot.shooter.getLocalAngle() + 10));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> robot.shooter.setLocalAngle(robot.shooter.getLocalAngle() - 10));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> robot.shooter.setLocalTargetTps(robot.shooter.getLocalTargetTps() + 100));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robot.shooter.setLocalTargetTps(robot.shooter.getLocalTargetTps() - 100));
    }

    @Override
    public void run() {
        robot.update();
        CommandScheduler.getInstance().run();

        if (!progressTransferCommand.isScheduled()) {
            double triggerValue = gamepad1.right_trigger - gamepad1.left_trigger;

            if (triggerValue != 0) {
                robot.intake.setCustomIntakeSpeed(triggerValue);
            } else {
                robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);
            }
        }

        panelsTelemetry.addData("Transfer Speed", TransferSubsystem.constantSpeed);

        panelsTelemetry.addData("Current Tps", robot.shooter.getCurrentTps());
        panelsTelemetry.addData("Target Tps", robot.shooter.getTargetTps());
        panelsTelemetry.addData("Shooter Motor State", robot.shooter.getShooterMotorState());
        panelsTelemetry.addData("Error", robot.shooter.getError());

        panelsTelemetry.addData("Angle", robot.shooter.getAngle());

        panelsTelemetry.addData("Distance To Goal", robot.localizer.getDistanceToGoal());

        panelsTelemetry.addData("Pose", robot.localizer.getPose());

        panelsTelemetry.addData("Transfer Error", robot.transfer.getError());
        panelsTelemetry.addData("Transfer State", robot.transfer.getTransferState());
        panelsTelemetry.addData("Intake State", robot.intake.getIntakeState());
        panelsTelemetry.addData("Progress", progressTransferCommand.isScheduled());
        panelsTelemetry.addData("Power", robot.transfer.transferMotor.get());
        panelsTelemetry.addData("MOTOR", robot.shooter.shooter1.get());

        panelsTelemetry.update(telemetry);
    }
}