package org.firstinspires.ftc.teamcode.drive;

import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class Drive extends MecanumDrive {
    private AutoDrive autoDrive;
    private ManualDrive manualDrive;

    public Drive(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight) {
        super(frontLeft, frontRight, backLeft, backRight);
    }

    // wrapper for two types of Driving
}
