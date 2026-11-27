package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.automations.ColorSensorManager;
import org.firstinspires.ftc.teamcode.automations.commands.ZeroTransferCommand;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.automations.odo.STATICLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class Robot {
    public IntakeSubsystem intake;
    public TransferSubsystem transfer;
    public ShooterSubsystem shooter;
    public STATICLocalizer localizer;
    public Drive drive;
    public HardwareMap hardwareMap;
    public ColorSensorManager colorSensorManager;

    public enum Team {
        RED,
        BLUE
    }

    public enum Subsystems {
        INTAKE,
        TRANSFER,
        SHOOTER,
        LOCALIZER,
        DRIVE
    }

    public Robot(HardwareMap hardwareMap, Team team, boolean resetEncoder, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        GlobalDataStorage.team = team;

        colorSensorManager = new ColorSensorManager(hardwareMap);

        // y is 6
        // x is 8
        if (team == Team.RED) {
            GlobalDataStorage.goalPose = new Pose(141.5 - 8, 141.5 - 6);
        }
        else if (team == Team.BLUE) {
            GlobalDataStorage.goalPose = new Pose(0 + 8, 141.5 - 6);
        }

        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.INTAKE) {
                intake = new IntakeSubsystem(hardwareMap);
            }
            if (subsystem == Subsystems.TRANSFER) {
                transfer = new TransferSubsystem(hardwareMap);

                if (resetEncoder) {
                    CommandScheduler.getInstance().schedule(new ZeroTransferCommand(transfer, true, 250));
                }
            }
            if (subsystem == Subsystems.SHOOTER) {
                shooter = new ShooterSubsystem(hardwareMap);
            }
            if (subsystem == Subsystems.LOCALIZER) {
                //if (GlobalDataStorage.staticLocalizer == null) {
                    localizer = new STATICLocalizer(hardwareMap, new Pose(72, 72, PI/2), STATICLocalizer.LocalizerMode.NO_ULTRASONICS);
                //} else {
                //    localizer = GlobalDataStorage.staticLocalizer;
                //}
            }
            if (subsystem == Subsystems.DRIVE) {
                drive = new Drive(hardwareMap, localizer);
            }
        }
    }

    public Robot(HardwareMap hardwareMap, Team team) {
        this(hardwareMap, team, false, Robot.Subsystems.INTAKE, Subsystems.TRANSFER, Subsystems.SHOOTER, Subsystems.LOCALIZER, Subsystems.DRIVE);
    }

    // TODO is this neccessay?????????????🥸🥸🥸
    public Robot(HardwareMap hardwareMap, Team team, boolean resetEncoder) {
        this(hardwareMap, team, true, Robot.Subsystems.INTAKE, Subsystems.TRANSFER, Subsystems.SHOOTER, Subsystems.LOCALIZER, Subsystems.DRIVE);
        this.drive.follower.setPose(GlobalDataStorage.robotPose);
    }

    public void update() {
        localizer.update();

        GlobalDataStorage.robotPose = localizer.getPose();

        shooter.updateDistanceToGoal(localizer.getDistanceToGoal());

        drive.update();
    }
}
