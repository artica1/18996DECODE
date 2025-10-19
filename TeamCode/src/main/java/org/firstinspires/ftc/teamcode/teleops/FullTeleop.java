package org.firstinspires.ftc.teamcode.teleops;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.HardwareMapNames;
import org.firstinspires.ftc.teamcode.Robot;

public class FullTeleop extends CommandOpMode {
    private Robot robot;

    private Motor frontLeft;
    private Motor frontRight;
    private Motor backLeft;
    private Motor backRight;
    private MecanumDrive drive;

    private GamepadEx gamepad;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        reset();
        initialize();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
    }

    @Override
    public void initialize() {
        Robot robot = new Robot(hardwareMap, Robot.Team.RED);

        frontLeft = hardwareMap.get(Motor.class, HardwareMapNames.DRIVE_FRONT_LEFT);
        frontRight = hardwareMap.get(Motor.class, HardwareMapNames.DRIVE_FRONT_RIGHT);
        backLeft = hardwareMap.get(Motor.class, HardwareMapNames.DRIVE_BACK_LEFT);
        backRight = hardwareMap.get(Motor.class, HardwareMapNames.DRIVE_BACK_RIGHT);
        drive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        gamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        drive.driveRobotCentric(gamepad.getLeftX(), gamepad.getLeftY(), gamepad.getRightY(), true);
    }

}
