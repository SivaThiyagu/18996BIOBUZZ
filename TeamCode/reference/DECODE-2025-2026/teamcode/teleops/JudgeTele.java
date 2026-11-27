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
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.commands.AdjustShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.automations.commands.HoldPointCommand;
import org.firstinspires.ftc.teamcode.automations.commands.ProgressTransferCommand;
import org.firstinspires.ftc.teamcode.automations.commands.ZeroTransferCommand;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

import java.util.List;

@Configurable
@TeleOp
public class JudgeTele extends CommandOpMode {
    private Robot robot;

    private GamepadEx gamepad;

    // Commands are single instanced for teleop
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
        robot = new Robot(hardwareMap, GlobalDataStorage.autoTeam, true);
        robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.UNPOWERED);

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

        // MANUAL ZERO
        gamepad.getGamepadButton(GamepadKeys.Button.OPTIONS)
                .whenPressed(() -> {
                    schedule(new ZeroTransferCommand(robot.transfer, true,2000));
                });

        // EDIT INSTANCED
        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> {
                    TransferSubsystem.constantSpeed = Range.clip(TransferSubsystem.constantSpeed += 0.1, 0, 1);
                });

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> {
                    TransferSubsystem.constantSpeed = Range.clip(TransferSubsystem.constantSpeed -= 0.1, 0, 1);
                });
    }

    @Override
    public void run() {
        robot.update();
        CommandScheduler.getInstance().run();

        telemetry.addData("Speed", TransferSubsystem.constantSpeed);
        telemetry.update();
    }
}
