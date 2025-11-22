package org.firstinspires.ftc.teamcode.automations.odo;

import androidx.annotation.NonNull;

import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.automations.pedroPathing.Constants;

public class STATICLocalizer implements Localizer {
    private final PinpointLocalizer pinpoint;
    private final LimelightManager limelightManager;
    private final UltrasonicsManager ultrasonicsManager;

    private LocalizerMode localizerMode;

    public enum LocalizerMode {
        All,
        NO_LIMELIGHT, // This only applies to localization, motif will still be scanned
        NO_ULTRASONICS,
        PINPOINT_ONLY
    }

    public STATICLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose());
    }

    public STATICLocalizer(HardwareMap hardwareMap, Pose setStartPose) {
        pinpoint = new PinpointLocalizer(hardwareMap, Constants.pinpointConstants, setStartPose);
        limelightManager = new LimelightManager(hardwareMap);
        ultrasonicsManager = new UltrasonicsManager(hardwareMap);

        setLocalizerMode(LocalizerMode.PINPOINT_ONLY);
    }

    @Override
    public Pose getPose() {
        return pinpoint.getPose();
    }

    @Override
    public Pose getVelocity() {
        return pinpoint.getVelocity();
    }

    @Override
    public Vector getVelocityVector() {
        return pinpoint.getVelocityVector();
    }

    @Override
    public void setStartPose(Pose setStart) {
        pinpoint.setStartPose(setStart);
    }

    @Override
    public void setPose(Pose setPose) {
        pinpoint.setPose(setPose);
    }

    @Override
    public void update() {
        // Keep "scanning" until something is found
        if(GlobalDataStorage.motif == null) {
            updateMotif();
        }

        pinpoint.update();

        if (localizerMode == LocalizerMode.All) {
            setPose(limelightManager.getPose(getIMUHeading()));
        }

        GlobalDataStorage.robotPose = getPose();
    }

    @Override
    public double getTotalHeading() {
        return pinpoint.getTotalHeading();
    }

    @Override
    public double getForwardMultiplier() {
        return Double.NaN;
    }

    @Override
    public double getLateralMultiplier() {
        return Double.NaN;
    }

    @Override
    public double getTurningMultiplier() {
        return Double.NaN;
    }

    @Override
    public void resetIMU() throws InterruptedException {
        pinpoint.resetIMU();
        limelightManager.reloadPipeline();
    }

    @Override
    public double getIMUHeading() {
        return pinpoint.getPose().getHeading();
    }

    @Override
    public boolean isNAN() {
        return pinpoint.isNAN();
    }

    // todo sanity check cuz ultrasonics might be tweaking
    public double getDistanceToGoal() {
        Pose currentPose = getPose();
        if (currentPose.distanceFrom(GlobalDataStorage.goalPose) > 60
                || Math.acos(currentPose.getHeadingAsUnitVector().dot(GlobalDataStorage.goalPose.getHeadingAsUnitVector())) > Math.toRadians(10)
                || currentPose.distanceFrom(GlobalDataStorage.goalPose) - ultrasonicsManager.getDistance() > 5
                || localizerMode == LocalizerMode.NO_ULTRASONICS
                || localizerMode == LocalizerMode.PINPOINT_ONLY
                || Double.isNaN(ultrasonicsManager.getDistance()))
        {
            return currentPose.distanceFrom(GlobalDataStorage.goalPose);
        } else {
            return ultrasonicsManager.getDistance();
        }
    }

    private void updateMotif() {
        GlobalDataStorage.motif = limelightManager.scanForMotif();
    }

    public void setLocalizerMode(@NonNull LocalizerMode localizerMode) {
        this.localizerMode = localizerMode;
    }

    public LocalizerMode getLocalizerMode() {
        return localizerMode;
    }
}
