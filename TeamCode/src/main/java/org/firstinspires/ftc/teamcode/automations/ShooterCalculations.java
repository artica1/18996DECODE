package org.firstinspires.ftc.teamcode.automations;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterCalculations {
    private static final InterpLUT angleILUT = new InterpLUT();
    private static final InterpLUT speedILUT = new InterpLUT();

    public static double getSpeed(double dist) {
        return speedILUT.get(dist);
    }

    public static double getAngle(double angle) {
        return angleILUT.get(angle);
    }

    public ShooterCalculations() {
        // add data


        speedILUT.createLUT();
        angleILUT.createLUT();
    }
}
