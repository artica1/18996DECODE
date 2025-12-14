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
        // X MUST BE INCREASING ORDER WHEN ADDING
        // IDFK WHY
        // otherwise it literally crashes the entire robot with no error message so dont do it

        // bruh
        angleILUT.add(-500, 10);
        speedILUT.add(-500, 950);

        // good ones
        angleILUT.add(20, 10);
        speedILUT.add(20, 950);

        angleILUT.add(30, 20);
        speedILUT.add(30, 1000);

        angleILUT.add(40, 35);
        speedILUT.add(40, 1050);

        angleILUT.add(50, 35);
        speedILUT.add(50, 1050);

        angleILUT.add(60, 40);
        speedILUT.add(60, 1150);

        angleILUT.add(70, 40);
        speedILUT.add(70, 1150);

        angleILUT.add(80, 40);
        speedILUT.add(80, 1200);

        angleILUT.add(90, 40);
        speedILUT.add(90, 1250);

        angleILUT.add(100, 45);
        speedILUT.add(100, 1250);

        angleILUT.add(110, 45);
        speedILUT.add(110, 1350);

        angleILUT.add(120, 45);
        speedILUT.add(120, 1450);

        angleILUT.add(130, 45);
        speedILUT.add(130, 1450);

        angleILUT.add(140, 45);
        speedILUT.add(140, 1450);
        // end of good

        // bruh
        angleILUT.add(500, 45);
        speedILUT.add(500, 1450);

        speedILUT.createLUT();
        angleILUT.createLUT();
    }
}
