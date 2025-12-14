package org.firstinspires.ftc.teamcode.automations.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
            .mass(10.5)
            .forwardZeroPowerAcceleration(-33.26)
            .lateralZeroPowerAcceleration(-72.65)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.08,
                    0,
                    0.01,
                    0.08
            ))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.01,
                    0.01
            ))
            .translationalPIDFSwitch(3)
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1,
                    0,
                    0.05,
                    0.02
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.03,
                    0,
                    0.001,
                    0.6,
                    0.03
            ))
            .centripetalScaling(0.0005)
            .holdPointHeadingScaling(1) // TODO wtf is this
            .holdPointTranslationalScaling(1);

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
            .xVelocity(84.25)
            .yVelocity(67.46)
            .useBrakeModeInTeleOp(true);

    public static PathConstraints pathConstraints = new PathConstraints(0.995, 100, 1.35, 1);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(92)
            .strafePodX(-50)
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