package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.automations.ColorSensorManager;

import java.util.Arrays;

@TeleOp
public class BareOpmode extends LinearOpMode {

    @Override
    public void runOpMode() {

        ColorSensorManager colorSensorManager = new ColorSensorManager(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Upper Distance", colorSensorManager.getUpperDistance());
            telemetry.addData("Middle Distance", colorSensorManager.getMiddleDistance());

            telemetry.addData("Upper Color", Arrays.toString(colorSensorManager.getUpperColor()));
            telemetry.addData("Middle Distance", Arrays.toString(colorSensorManager.getMiddleColor()));

            telemetry.addData("Ball Detected", colorSensorManager.ballDetected());
            telemetry.update();
        }
    }
}
