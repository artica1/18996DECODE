package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareMapNames;

@Configurable
public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakeMotor;

    private IntakeState intakeState;
    private double velocity;

    public enum IntakeState {
        OUTTAKE,
        HOLD,
        CUSTOM,
        INTAKE,
        REVERSE,
        DISABLED;

        public double getValue() {
            switch (this) {
                case OUTTAKE:
                    return 0.75;
                case HOLD:
                    return 0.1;
                case INTAKE:
                    return 1.0;
                case REVERSE:
                    return -1.0;
                case CUSTOM:
                case DISABLED:
                    return 0.1;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = new MotorEx(hardwareMap, HardwareMapNames.INTAKE_MOTOR, Motor.GoBILDA.RPM_1150);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.set(0);

        setIntakeState(IntakeState.DISABLED);
    }

    @Override
    public void periodic() {
        if (intakeState.equals(IntakeState.CUSTOM)) {
            intakeMotor.setVelocity(velocity * 83.333, AngleUnit.RADIANS); // chatgpt made this conversion value
        }
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;

        intakeMotor.set(intakeState.getValue());
    }

    public void setCustomIntakeVelocity(double velocity) {
        this.intakeState = IntakeState.CUSTOM;

        this.velocity = velocity;
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }
}
