package org.firstinspires.ftc.teamcode.automations.odo;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.Motif;

import java.util.List;

public class LimelightManager {
    private final Limelight3A limelight;

    public LimelightManager(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, HardwareMapNames.LIMELIGHT);
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // general AprilTag
        limelight.start();
    }

    // TODO check for degrees/radians in this
    public Pose getPose(double heading) {
        LLResult result = limelight.getLatestResult();
        limelight.updateRobotOrientation(heading);

        if (result != null && result.isValid() && result.getStaleness() < 100) {

            Pose3D llPose = result.getBotpose_MT2();

            if (llPose != null) {
                return new Pose(llPose.getPosition().x, llPose.getPosition().y, heading);
            }
            else {
                return null;
            }
        } else {
            return null;
        }
    }

    public LLStatus getStatus() {
        return limelight.getStatus();
    }

    public void reloadPipeline() {
        limelight.reloadPipeline();
    }

    public Motif scanForMotif() {
        List<LLResultTypes.FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            int id = fiducial.getFiducialId();

            if (id == 21) {
                return Motif.GPP;
            }
            if (id == 22) {
                return Motif.PGP;
            }
            if (id == 23) {
                return Motif.PPG;
            }
            else {
                return null;
            }
        }
        return null;
    }
}
