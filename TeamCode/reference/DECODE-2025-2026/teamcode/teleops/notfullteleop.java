package org.firstinspires.ftc.teamcode.teleops;

import static org.firstinspires.ftc.teamcode.Robot.Team.BLUE;
import static org.firstinspires.ftc.teamcode.Robot.Team.RED;
import static java.lang.Math.PI;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.commands.AdjustShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.automations.commands.ProgressTransferCommand;
import org.firstinspires.ftc.teamcode.automations.commands.ZeroTransferCommand;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.List;

@Configurable
@TeleOp
public class notfullteleop extends CommandOpMode {
    private Robot robot;

    private GamepadEx gamepad;
    private GamepadEx gamepad_2;

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

        panelsTelemetry.addData("Team", GlobalDataStorage.autoTeam);
        panelsTelemetry.update();

        robot = new Robot(hardwareMap, GlobalDataStorage.autoTeam, true);

        gamepad = new GamepadEx(gamepad1);

        gamepad_2 = new GamepadEx(gamepad2);

        /*
        // FIRE ONE
        gamepad.getGamepadButton(GamepadKeys.Button.SQUARE)
                .whenPressed(() -> {
                    progressTransferCommand = new ProgressTransferCommand(robot);
                    schedule(progressTransferCommand);
                });

         */

        // MANUAL ZERO
        gamepad.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(() -> {
                    schedule(new ZeroTransferCommand(robot.transfer, true,2000));
                });

        gamepad.getGamepadButton(GamepadKeys.Button.SHARE)
                .whenPressed(() -> {
                    if (GlobalDataStorage.team == RED) robot.drive.follower.setPose(new Pose(9.5, 8.5, 0));
                    else if (GlobalDataStorage.team == BLUE) robot.drive.follower.setPose(new Pose(135.1, 8.4, PI));
                });

        /*
        // TRANSLATIONAL LOCK
        gamepad.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(() -> {
                    holdPointCommand = new HoldPointCommand(robot);
                    schedule(holdPointCommand);
                }).whenReleased(() -> {
                    holdPointCommand.end(false);
                });

         */

        // HEADING LOCK
        gamepad_2.getGamepadButton(GamepadKeys.Button.CROSS)
                .whenPressed(() -> {
                    robot.drive.setHeadingLock(true);
                    adjustShooterSpeedCommand = new AdjustShooterSpeedCommand(robot);
                    schedule(adjustShooterSpeedCommand.perpetually());
                })
                .whenReleased(() -> {
                    robot.drive.setHeadingLock(false);
                    adjustShooterSpeedCommand.cancel();
                });

        // FIRE ALL
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> {
                    progressTransferCommand = new ProgressTransferCommand(robot, true);
                    schedule(progressTransferCommand);
                });

        // IDLE/RESET
        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> {
                    holdPointCommand.cancel();
                    adjustShooterSpeedCommand.cancel();
                    progressTransferCommand.cancel();

                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);
                    robot.shooter.setShooterAngleState(ShooterSubsystem.ShooterAngleState.AUTO);
                    robot.drive.setDriveMode(Drive.DriveMode.MANUAL);
                    robot.drive.setHeadingLock(false);
                    robot.drive.setTranslationLock(false);
                });

        /*
        // SET SPEED AND ANGLE, CLOSE LAUNCH ZONE (Left Back Paddle)
        gamepad.getGamepadButton(GamepadKeys.Button.CIRCLE)
                .whenPressed(() -> {
                    robot.shooter.setTargetTps(1600);
                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.ACTIVE);
                });

        // SET SPEED AND ANGLE, FAR LAUNCH ZONE
        // todo
        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> {
                    if (GlobalDataStorage.team == BLUE) robot.drive.follower.setPose(new Pose(19.9, 120.6, Math.toRadians(141.7)));
                    else if (GlobalDataStorage.team == RED) robot.drive.follower.setPose(new Pose(19.9, 120.6, Math.toRadians(141.7)).mirror());
                });

        // FINE DRIVE ENABLE
        gamepad.getGamepadButton(GamepadKeys.Button.TRIANGLE)
                .whenPressed(() -> robot.drive.setDriveMode(Drive.DriveMode.FINE));

        // EDIT INSTANCED
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> robot.shooter.setLocalAngle(robot.shooter.getLocalAngle() + 5));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> robot.shooter.setLocalAngle(robot.shooter.getLocalAngle() - 5));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> robot.shooter.setLocalTargetTps(robot.shooter.getLocalTargetTps() + 100));

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> robot.shooter.setLocalTargetTps(robot.shooter.getLocalTargetTps() - 100));

         */
    }

    @Override
    public void run() {
        robot.update();
        CommandScheduler.getInstance().run();

        robot.drive.setTeleOpVectors(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        if (!progressTransferCommand.isScheduled()) {
            double triggerValue = gamepad1.right_trigger - gamepad1.left_trigger;

            if (triggerValue != 0) {
                robot.intake.setCustomIntakeSpeed(triggerValue);
            } else {
                robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);
            }
        }

        /*
        if (Math.abs(robot.shooter.getError()) < 10 && previousVelocityError > 50) {
            gamepad1.rumbleBlips(1);
        }
        previousVelocityError = Math.abs(robot.shooter.getError());

         */

        /*
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

        panelsTelemetry.addData("headingLock", robot.drive.isHeadingLock());
        panelsTelemetry.addData("translationalLock", robot.drive.isTranslationLock());

         */

        panelsTelemetry.update(telemetry);
    }
}
