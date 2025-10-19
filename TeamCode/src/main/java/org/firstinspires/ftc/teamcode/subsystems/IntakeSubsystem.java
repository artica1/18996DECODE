package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

public class IntakeSubsystem extends SubsystemBase {

    private final MotorEx motor;
    private double velocity;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, HardwareMapNames.INTAKE_MOTOR, Motor.GoBILDA.RPM_312); //todo change to match gear ratio?
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor.setVeloCoefficients(1, 0, 0);
        motor.setFeedforwardCoefficients(0, 1, 0);
    }

    @Override
    public void periodic() {
        motor.setVelocity(velocity/0.1508, AngleUnit.RADIANS);
    }

    public void setTangentialVelocity(double velocity) {
        this.velocity = velocity;
    }

    public void holdMotor() {
        this.velocity = 1 / (2 * PI);
    }
}
