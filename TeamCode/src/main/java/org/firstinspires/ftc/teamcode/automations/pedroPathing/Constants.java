package org.firstinspires.ftc.teamcode.automations.pedroPathing;

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

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName(HardwareMapNames.DRIVE_FRONT_RIGHT)
            .rightRearMotorName(HardwareMapNames.DRIVE_BACK_RIGHT)
            .leftRearMotorName(HardwareMapNames.DRIVE_BACK_LEFT)
            .leftFrontMotorName(HardwareMapNames.DRIVE_FRONT_LEFT)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .forwardPodY(92)
            .strafePodX(-50)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName(HardwareMapNames.PINPOINT)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap, Localizer localizer) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .setLocalizer(localizer)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    public static Follower createFollower(HardwareMap hardwareMap) {
        return createFollower(hardwareMap, new PinpointLocalizer(hardwareMap, pinpointConstants));
    }
}
