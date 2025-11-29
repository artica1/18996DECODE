package org.firstinspires.ftc.teamcode.automations.drive;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.automations.pedroPathing.Constants;

public class Drive {
    public final Follower follower;
    private DriveMode driveMode;
    private Pose holdPoint;

    public enum DriveMode {
        MANUAL,
        FINE,
        HOLD_POINT,
        AUTO
    }

    public Drive(HardwareMap hardwareMap, Localizer localizer) {
        this(hardwareMap, localizer, DriveMode.MANUAL);
    }

    public Drive(HardwareMap hardwareMap, Localizer localizer, DriveMode driveMode) {
        follower = Constants.createFollower(hardwareMap, localizer);

        setDriveMode(driveMode);
    }

    public void update() {
        follower.update();

        GlobalDataStorage.robotPose = follower.getPose();
    }

    // todo rate profiling
    // todo fine control
    public void setTeleOpVectors(double x, double y, double h) {
        follower.setTeleOpDrive(-y, -x, -h);
    }

    public void setDriveMode(@NonNull DriveMode driveMode) {
        this.driveMode = driveMode;

        if (driveMode == DriveMode.MANUAL || driveMode == DriveMode.FINE) {
            follower.startTeleopDrive(true);
        } else if (driveMode == DriveMode.HOLD_POINT) {
            follower.holdPoint(new BezierPoint(holdPoint), holdPoint.getHeading(), false);
        }
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setHoldPoint(Pose holdPoint) {
        this.holdPoint = holdPoint;
    }
}
