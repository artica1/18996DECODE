package org.firstinspires.ftc.teamcode.automations.odo;

import static java.lang.Math.atan;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMapNames;

public class UltrasonicsManager {
    private final AnalogInput leftUltrasonic;
    private final AnalogInput rightUltrasonic;

    public UltrasonicsManager(HardwareMap hardwareMap) {
        leftUltrasonic = hardwareMap.get(AnalogInput.class, HardwareMapNames.LEFT_ULTRASONIC);
        rightUltrasonic = hardwareMap.get(AnalogInput.class, HardwareMapNames.RIGHT_ULTRASONIC);
    }

    public double getDistance() {
        return getDistanceFromVoltage((leftUltrasonic.getVoltage() + rightUltrasonic.getVoltage()) / 2);
    }

    // VERY approximate
    // This probably sucks
    public double getAngle() {
        return atan(getDistanceFromVoltage(rightUltrasonic.getVoltage() - leftUltrasonic.getVoltage()) / 4); // TODO GET REAL VALUE
    }

    // Returns inches
    private static double getDistanceFromVoltage(double voltage) {
        return voltage / 3.3 * 520 / 2.54;
    }
}
