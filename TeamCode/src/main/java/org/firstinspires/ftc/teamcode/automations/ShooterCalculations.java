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
        angleILUT.add(-500, 5);
        speedILUT.add(-500, 1100);

        angleILUT.add(20, 5);
        speedILUT.add(20, 1100);

        angleILUT.add(40, 10);
        speedILUT.add(40, 1100);

        angleILUT.add(60, 40);
        speedILUT.add(60, 1180);

        angleILUT.add(70, 40);
        speedILUT.add(70, 1260);

        angleILUT.add(75, 40);
        speedILUT.add(75, 1300);

        angleILUT.add(100, 45);
        speedILUT.add(100, 1460);

        angleILUT.add(140, 45);
        speedILUT.add(140, 1650);

        // bruh
        angleILUT.add(500, 45);
        speedILUT.add(500, 1700);


        speedILUT.createLUT();
        angleILUT.createLUT();
    }
}
