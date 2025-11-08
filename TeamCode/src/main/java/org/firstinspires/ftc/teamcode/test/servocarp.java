package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.ServoEx;

import org.firstinspires.ftc.teamcode.HardwareMapNames;

@TeleOp
public class servocarp extends LinearOpMode {

    @Override
    public void runOpMode() {

        ServoEx shooterServo = new ServoEx(hardwareMap, HardwareMapNames.SHOOTER_SERVO);

        shooterServo.set(0.5);

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
