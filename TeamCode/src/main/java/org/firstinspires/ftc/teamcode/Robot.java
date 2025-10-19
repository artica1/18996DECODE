package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TransferSubsystem;

public class Robot {
    public IntakeSubsystem intake;
    public TransferSubsystem transfer;
    public ShooterSubsystem shooter;
    public HardwareMap hardwareMap;
    private final Team team;

    public enum Team {
        RED,
        BLUE
    }

    public enum Subsystems {
        INTAKE,
        TRANSFER,
        SHOOTER
    }

    public Robot(HardwareMap hardwareMap, Team team, Subsystems... subsystems) {
        this.hardwareMap = hardwareMap;
        this.team = team;

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
        }
    }

    public Robot(HardwareMap hardwareMap, Team team) {
        this(hardwareMap, team, Robot.Subsystems.INTAKE, Subsystems.TRANSFER, Subsystems.SHOOTER);
    }

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, Team.RED);
    }

    public Team getTeam() {
        return team;
    }
}
