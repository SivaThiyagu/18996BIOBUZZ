package org.firstinspires.ftc.teamcode.automations.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.3)
            /*
            .forwardZeroPowerAcceleration(-37.5425)
            .lateralZeroPowerAcceleration(-72.8025)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.015, 0.1))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.00, 0.02))
            .translationalPIDFSwitch(1)

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.08))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.00, 0.00))
            .headingPIDFSwitch(0.0270796327)

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.0012, 0.6, 0.03))
            .useSecondaryDrivePIDF(false) // KEEP THIS FRICKING FALSE
            .drivePIDFSwitch(20)

            .centripetalScaling(0.0005);
             */

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.1, 0.08))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.00, 0.00))
            .headingPIDFSwitch(0.0270796327)

            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.09, 0.061601432884104655, 0.001797718365332371))

            .centripetalScaling(0);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(HardwareMapNames.DRIVE_FRONT_RIGHT)
            .rightRearMotorName(HardwareMapNames.DRIVE_BACK_RIGHT)
            .leftRearMotorName(HardwareMapNames.DRIVE_BACK_LEFT)
            .leftFrontMotorName(HardwareMapNames.DRIVE_FRONT_LEFT)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(75.635)
            .yVelocity(58.025)
            .useBrakeModeInTeleOp(true);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 1, 1);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(-92)
            .strafePodX(-48)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(HardwareMapNames.PINPOINT)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setLocalizer(localizer)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    // for compatibility w/ pedro tuning
    public static Follower createFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap, new PinpointLocalizer(hardwareMap, pinpointConstants));
    }
}