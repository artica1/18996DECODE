package org.firstinspires.ftc.teamcode.automations.odo;

import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.lang.Math;

public class ColorSensorManager {
    private ColorSensor upper, middle;
    public ColorSensorManager(HardwareMap hardwareMap) {
        upper = hardwareMap.get(RevColorSensorV3.class, HardwareMapNames.UPPER_COLOR);
        middle = hardwareMap.get(RevColorSensorV3.class, HardwareMapNames.MIDDLE_COLOR);
    }

    // r g b a
    double[] purple = {0, 0, 0, 0};
    double[] green = {0, 0, 0, 0};
    double[] ambient = {0, 0, 0, 0};

    public Colors getColor(double[] point) {
        double pd = calcDistance(point, purple);
        double gd = calcDistance(point, green);
        double ad = calcDistance(point, ambient);

        if (pd < ad) {
            if (pd < gd) {
                return Colors.PURPLE;
            }
            else {
                return Colors.GREEN;
            }
        }
        else {
            if (ad < gd) {
                return Colors.AMBIENT;
            }
            else {
                return Colors.GREEN;
            }
        }
    }

    private double calcDistance(double[] point, double[] ref) {
        double r, g, b, a;
        r = point[0] - ref[0];
        g = point[1] - ref[1];
        b = point[2] - ref[2];
        a = point[3] - ref[3];

        double ds = r * r + g * g + b * b + a * a;
        return Math.pow(ds, 0.5);
    }


}
