package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class GlobalDataStorage {
    // EX: public static Pose2D robotPose = new Pose2D();

    // general static class for passing data between OpModes, like the decoded value of the obelisk, or robot ending positions after auto, etc.

    public static Motif motif = null;
    public static Robot.Team team;
    public static Pose goalPose;
    public static Pose robotPose;
    public static double angleTrim = 0;
    public static int velocityTrim = 0;
}
