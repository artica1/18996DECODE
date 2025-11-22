package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.HardwareMapNames;

@TeleOp
public class BareOpmode extends LinearOpMode {

    @Override
    public void runOpMode() {

        MotorEx shooterServo = new MotorEx(hardwareMap, HardwareMapNames.SHOOTER_MOTOR, Motor.GoBILDA.BARE);

        waitForStart();

        while (opModeIsActive()) {
            shooterServo.set(1);

        }
    }
}
