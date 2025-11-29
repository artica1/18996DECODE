package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

import org.firstinspires.ftc.teamcode.HardwareMapNames;

@TeleOp
public class BareOpmode extends LinearOpMode {

    @Override
    public void runOpMode() {

        InterpLUT lut = new InterpLUT();

        lut.add(30, 30);
        lut.add(100, 45);
        lut.createLUT();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Output", lut.get(50));
            telemetry.update();
        }
    }
}
