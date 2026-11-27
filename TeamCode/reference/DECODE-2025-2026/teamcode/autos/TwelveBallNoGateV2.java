package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.commands.AdjustShooterSpeedCommand;
import org.firstinspires.ftc.teamcode.automations.commands.AutoShootCommand;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

@Autonomous
public class TwelveBallNoGateV2 extends OpMode {
    private Robot robot;

    // gate hit
    // 25.6 56 155

    // gate prepare
    // 30.7 53.5 155

    //

    private Timer pathTimer, autoTimer;
    private int pathState;

    private Pose startPose, shootPose, intakeMark1StartPose, intakeMark1EndPose, intakeMark2StartPose, intakeMark2EndPose, intakeMark3StartPose, intakeMark3EndPose, parkPose, gatePreparePose, gateContactPose, gate1, gate2, openGate;

    public PathChain shootPreload, prepareMark1, intakeMark1, shootMark1, prepareMark2, intakeMark2, shootMark2, prepareMark3, intakeMark3, shootMark3, park, gatePrepare, gateContact, gateToShoot, shootToGate;

    private Command autoShootCommand = new InstantCommand();
    private Command adjustShooterSpeedCommand = new InstantCommand();
    private Command holdPointCommand = new InstantCommand();

    @Override
    public void init() {
        pathTimer = new Timer();
        autoTimer = new Timer();

        CommandScheduler.getInstance().reset();

        GlobalDataStorage.staticLocalizer = null;
        robot = new Robot(hardwareMap, GlobalDataStorage.autoTeam, true);

        buildPoses(GlobalDataStorage.autoTeam);
        buildPaths();

        robot.drive.setDriveMode(Drive.DriveMode.AUTO);
        robot.localizer.setPose(startPose);

        robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);

        telemetry.addData("x", robot.localizer.getPose().getX());
        telemetry.addData("y", robot.localizer.getPose().getY());
        telemetry.addData("heading", robot.localizer.getPose().getHeading());
        telemetry.addData("Team", GlobalDataStorage.autoTeam);
        telemetry.update();
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        robot.update();
        CommandScheduler.getInstance().run();

        GlobalDataStorage.robotPose = robot.localizer.getPose();

        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", robot.localizer.getPose().getX());
        telemetry.addData("y", robot.localizer.getPose().getY());
        telemetry.addData("heading", robot.localizer.getPose().getHeading());
        telemetry.addData("busy", robot.drive.follower.isBusy());
        telemetry.addData("error", robot.shooter.getError());
        telemetry.addData("Transfer", robot.transfer.getTransferState());
        telemetry.addData("Transfer Error", robot.transfer.getError());
        telemetry.update();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                robot.drive.follower.followPath(shootPreload, true);
                robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                adjustShooterSpeedCommand = new AdjustShooterSpeedCommand(robot, shootPose);
                adjustShooterSpeedCommand.schedule();

                autoShootCommand = new AutoShootCommand(robot);
                autoShootCommand.schedule();

                setPathState(7);
                break;
            case 2:
                if(autoShootCommand.isFinished()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(prepareMark1, false);
                    setPathState(3);
                }
                break;
            case 3:
                if (!robot.drive.follower.isBusy()) {

                    robot.drive.follower.followPath(intakeMark1, false);
                    setPathState(50);
                }
                break;
            case 50:
                if (!robot.drive.follower.isBusy()) {

                    robot.drive.follower.followPath(gatePrepare, false);
                    setPathState(51);
                }
                break;
            case 51:
                if (!robot.drive.follower.isBusy()) {

                    robot.drive.follower.followPath(gateContact, false);
                    setPathState(4);
                }
                break;
            case 4:
                if(!robot.drive.follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    robot.drive.follower.followPath(shootMark1, true);

                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    setPathState(6);
                }
                break;
            case 6:
                if(autoShootCommand.isFinished()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(prepareMark2, false);
                    setPathState(10);
                }
                break;
            case 7:
                if (!robot.drive.follower.isBusy()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(intakeMark2, false);
                    setPathState(8);
                }
                break;
            case 8:
                if(!robot.drive.follower.isBusy()) {

                    robot.drive.follower.followPath(shootMark2, true);

                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    setPathState(15);
                }
                break;
            case 10:
                if(autoShootCommand.isFinished()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    robot.drive.follower.followPath(prepareMark3, false);
                    setPathState(11);
                }
                break;
            case 11:
                if (!robot.drive.follower.isBusy()) {

                    robot.drive.follower.followPath(intakeMark3, false);
                    setPathState(12);
                }
                break;
            case 12:
                if(!robot.drive.follower.isBusy()) {

                    robot.drive.follower.followPath(shootMark3);

                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    setPathState(14);
                }
                break;
            case 14:
                if(autoShootCommand.isFinished()) {
                    robot.drive.follower.followPath(park);

                    robot.shooter.setShooterMotorState(ShooterSubsystem.ShooterMotorState.IDLE);

                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.DISABLED);
                    setPathState(-1);
                }
                break;
            case 15:
                if(!robot.drive.follower.isBusy()) {
                    robot.drive.follower.followPath(shootToGate, true);
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.INTAKE);

                    setPathState(16);
                }
                break;
            case 16:
                if (!robot.drive.follower.isBusy()) {
                    robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD);
                    robot.drive.follower.followPath(gateToShoot, true);

                    setPathState(17);
                }
                break;
            case 17:
                if (!robot.drive.follower.isBusy()) {
                    autoShootCommand = new AutoShootCommand(robot);
                    autoShootCommand.schedule();

                    setPathState(18);
                }
                break;
            case 18:
                if (autoShootCommand.isFinished()) {
                    setPathState(3);
                }
                break;
        }
    }

    public void buildPaths() {
        shootPreload = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                startPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .addParametricCallback(0.3, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))
                .build();

        prepareMark1 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose,
                                intakeMark1StartPose
                        )
                ).setTangentHeadingInterpolation()
                .build();

        intakeMark1 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeMark1StartPose,
                                intakeMark1EndPose
                        )
                ).setTangentHeadingInterpolation()
                .build();

        shootMark1 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                gateContactPose,
                                shootPose
                        )
                )
                .setLinearHeadingInterpolation(intakeMark1EndPose.getHeading(), shootPose.getHeading())
                .addParametricCallback(0.3, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))
                .build();

        prepareMark2 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose,
                                intakeMark2StartPose
                        )
                )
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakeMark2StartPose.getHeading())
                .build();

        intakeMark2 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeMark2StartPose,
                                intakeMark2EndPose
                        )
                ).setLinearHeadingInterpolation(intakeMark2StartPose.getHeading(), intakeMark2EndPose.getHeading())
                .build();

        shootMark2 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeMark2EndPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(intakeMark2EndPose.getHeading(), shootPose.getHeading())
                .addParametricCallback(0.3, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))
                .build();

        prepareMark3 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose,
                                intakeMark3StartPose
                        )
                ).setLinearHeadingInterpolation(shootPose.getHeading(), intakeMark3StartPose.getHeading())
                .build();

        intakeMark3 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeMark3StartPose,
                                intakeMark3EndPose
                        )
                ).setLinearHeadingInterpolation(intakeMark3StartPose.getHeading(), intakeMark3EndPose.getHeading())
                .build();

        shootMark3 = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeMark3EndPose,
                                shootPose
                        )
                ).setLinearHeadingInterpolation(intakeMark3EndPose.getHeading(), shootPose.getHeading())
                .addParametricCallback(0.3, ()-> robot.intake.setIntakeState(IntakeSubsystem.IntakeState.HOLD))
                .build();

        park = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                shootPose,
                                parkPose
                        )
                ).setConstantHeadingInterpolation(parkPose.getHeading())
                .build();

        gatePrepare = robot.drive.follower.pathBuilder().addPath(
                        new BezierLine(
                                intakeMark1EndPose,
                                gatePreparePose
                        )
                ).setConstantHeadingInterpolation(gatePreparePose.getHeading())
                .build();

        gateContact = robot.drive.follower.pathBuilder().addPath(
                        new BezierCurve(
                                gatePreparePose,
                                gate1,
                                gate2,
                                gateContactPose
                        )
                ).setConstantHeadingInterpolation(gateContactPose.getHeading())
                .build();

        shootToGate = robot.drive.follower.pathBuilder().addPath(
                new BezierLine(
                    shootPose, openGate
                )
            ).setTangentHeadingInterpolation().build();

        gateToShoot = robot.drive.follower.pathBuilder().addPath(
                new BezierLine(
                        openGate, shootPose
                )
            ).setTangentHeadingInterpolation().build();
    }

    public void buildPoses(Robot.Team team) {
        // BUILT FOR BLUE
        startPose = new Pose(19.9, 120.6, Math.toRadians(141.7));
        // startPose = new Pose(56, 10, Math.toRadians(90));

        shootPose = new Pose(55.8, 83.5, Math.toRadians(132));
        // 20.3 75.1 90

        intakeMark1StartPose = new Pose(42, 84, Math.toRadians(180));

        intakeMark1EndPose = new Pose(18, 84, Math.toRadians(180));

        intakeMark2StartPose = new Pose(54, 58, Math.toRadians(180));

        intakeMark2EndPose = new Pose(10, 56, Math.toRadians(180));

        intakeMark3StartPose = new Pose(52, 36, Math.toRadians(180));

        intakeMark3EndPose = new Pose(10, 36, Math.toRadians(180));

        gatePreparePose = new Pose(24, 84, Math.toRadians(90));

        gateContactPose = new Pose(15.3, 75.1, Math.toRadians(90));

        parkPose = new Pose(42, 84, Math.toRadians(135));



        gate1 = new Pose(26.44312796208532, 79.2867298578199);
        gate2 = new Pose(22.17298578199052, 74.77962085308057);

        openGate = new Pose(11.9, 60.3, 2.6417);

        if (team == Robot.Team.RED) {
            startPose = startPose.mirror();

            shootPose = shootPose.mirror();

            intakeMark1StartPose = intakeMark1StartPose.mirror();
            intakeMark1EndPose = intakeMark1EndPose.mirror();

            intakeMark2StartPose = intakeMark2StartPose.mirror();
            intakeMark2EndPose = intakeMark2EndPose.mirror();

            intakeMark3StartPose = intakeMark3StartPose.mirror();
            intakeMark3EndPose = intakeMark3EndPose.mirror();

            gatePreparePose = gatePreparePose.mirror();
            gateContactPose = gateContactPose.mirror();

            gate1 = gate1.mirror();
            gate2 = gate2.mirror();

            parkPose = parkPose.mirror();
        }

        // built for red mirrors
        // startPose.mirror();
    }

    public void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}
