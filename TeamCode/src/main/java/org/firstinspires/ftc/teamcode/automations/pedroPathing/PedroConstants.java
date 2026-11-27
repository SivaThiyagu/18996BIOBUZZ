package org.firstinspires.ftc.teamcode.automations.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tooling.HardwareMapNames;

public class PedroConstants {
    public static boolean ROBOT_SETTINGS_VERIFIED = false;

    public static FollowerConstants follower = new FollowerConstants()
            .mass(12.0);

    public static PinpointConstants pinpoint = new PinpointConstants()
            .hardwareMapName(HardwareMapNames.PINPOINT)
            .forwardPodY(0.0)
            .strafePodX(0.0)
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static MecanumConstants drivetrain = new MecanumConstants()
            .leftFrontMotorName(HardwareMapNames.DRIVE_FRONT_LEFT)
            .rightFrontMotorName(HardwareMapNames.DRIVE_FRONT_RIGHT)
            .leftRearMotorName(HardwareMapNames.DRIVE_BACK_LEFT)
            .rightRearMotorName(HardwareMapNames.DRIVE_BACK_RIGHT)

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .maxPower(0.25);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(follower, hardwareMap)
                .mecanumDrivetrain(drivetrain)
                .pinpointLocalizer(pinpoint)
                .build();
    }
}
