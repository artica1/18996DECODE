package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.automations.ColorSensorManager;
import org.firstinspires.ftc.teamcode.automations.drive.Drive;
import org.firstinspires.ftc.teamcode.automations.odo.STATICLocalizer;
import org.firstinspires.ftc.teamcode.automations.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class Robot {
    public IntakeSubsystem intake;
    public TransferSubsystem transfer;
    public ShooterSubsystem shooter;
    public STATICLocalizer localizer;
    public Drive drive;
    public HardwareMap hardwareMap;
    public ColorSensorManager colorSensorManager;

    public enum Team {
        RED,
        BLUE
    }

    public enum Subsystems {
        INTAKE,
        TRANSFER,
        SHOOTER,
        LOCALIZER,
        DRIVE
    }

    public Robot(HardwareMap hardwareMap, Team team, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        GlobalDataStorage.team = team;

        colorSensorManager = new ColorSensorManager(hardwareMap);

        if (team == Team.RED) {
            GlobalDataStorage.goalPose = new Pose(144, 144);
        }
        else if (team == Team.BLUE) {
            GlobalDataStorage.goalPose = new Pose(0, 144);
        }

        for (Subsystems subsystem : subsystems) {
            if (subsystem == Subsystems.INTAKE) {
                intake = new IntakeSubsystem(hardwareMap);
            }
            if (subsystem == Subsystems.TRANSFER) {
                transfer = new TransferSubsystem(hardwareMap);
            }
            if (subsystem == Subsystems.SHOOTER) {
                shooter = new ShooterSubsystem(hardwareMap);
            }
            if (subsystem == Subsystems.LOCALIZER) {
                localizer = new STATICLocalizer(hardwareMap);
            }
            if (subsystem == Subsystems.DRIVE) {
                drive = new Drive(hardwareMap, localizer);
            }
        }
    }

    public Robot(HardwareMap hardwareMap, Team team) {
        this(hardwareMap, team, Robot.Subsystems.INTAKE, Subsystems.TRANSFER, Subsystems.SHOOTER, Subsystems.LOCALIZER, Subsystems.DRIVE);
    }

    public void update() {
        localizer.update();
        drive.update();

        shooter.updateDistanceToGoal(localizer.getDistanceToGoal());
    }
}
