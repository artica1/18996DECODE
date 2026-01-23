package org.firstinspires.ftc.teamcode.automations;

import com.seattlesolvers.solverslib.util.InterpLUT;

public class ShooterCalculations {
    private static final InterpLUT angleILUT = new InterpLUT();
    private static final InterpLUT speedILUT = new InterpLUT();
    private static final InterpLUT transferILUT = new InterpLUT();

    public static double getSpeed(double dist) {
        return speedILUT.get(dist);
    }

    public static double getAngle(double dist) {
        return angleILUT.get(dist);
    }

    public static double getTransferSpeed(double dist) {
        return transferILUT.get(dist);
    }

    static {
        // X MUST BE INCREASING ORDER WHEN ADDING
        // otherwise it literally crashes the entire robot with no error message so dont do it

        // bruh
        angleILUT.add(-500, 10);
        speedILUT.add(-500, 950);
        transferILUT.add(-500, 1.0);

        // good ones
        angleILUT.add(25, 15);
        speedILUT.add(25, 1000);
        transferILUT.add(25, 1.0);

        angleILUT.add(35, 20);
        speedILUT.add(35, 1050);
        transferILUT.add(35, 1.0);

        angleILUT.add(45, 25);
        speedILUT.add(45, 1050);
        transferILUT.add(45, 0.6);

        angleILUT.add(55, 30);
        speedILUT.add(55, 1150);
        transferILUT.add(55, 0.6);

        angleILUT.add(65, 30);
        speedILUT.add(65, 1150);
        transferILUT.add(65, 0.6);

        angleILUT.add(75, 35);
        speedILUT.add(75, 1250);
        transferILUT.add(75, 0.5);

        angleILUT.add(80, 40);
        speedILUT.add(80, 1200);
        transferILUT.add(80, 0.5);

        angleILUT.add(90, 40);
        speedILUT.add(90, 1250);
        transferILUT.add(90, 0.4);

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

        // ???
        angleILUT.add(500, 45);
        speedILUT.add(500, 1450);
        transferILUT.add(500, 0.4);

        speedILUT.createLUT();
        angleILUT.createLUT();
        transferILUT.createLUT();
    }
}
