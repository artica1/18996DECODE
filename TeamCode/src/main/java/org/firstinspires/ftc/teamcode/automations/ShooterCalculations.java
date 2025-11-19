package org.firstinspires.ftc.teamcode.automations;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterCalculations {
    private static final InterpLUT angleILUT = new InterpLUT();
    private static final InterpLUT speedILUT = new InterpLUT();

    public static double getSpeed(double dist) {
        return speedILUT.get(dist);
    }

    public static double getAngle(double dist) {
        return angleILUT.get(dist);
    }

    static {
        angleILUT.add(10, 0.7);
        speedILUT.add(10, 1400);
        // add data
        speedILUT.createLUT();
        angleILUT.createLUT();
    }
}
