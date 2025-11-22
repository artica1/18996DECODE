package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.GlobalDataStorage;
import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.automations.odo.UltrasonicsManager;

@TeleOp
public class ultrasonicstet extends LinearOpMode {

    @Override
    public void runOpMode() {

        UltrasonicsManager ultrasonicsManager = new UltrasonicsManager(hardwareMap);
        GlobalDataStorage.team = Robot.Team.BLUE;

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance", ultrasonicsManager.getDistance());
            telemetry.update();
        }
    }
}
