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
    public static double OUTTAKE_SPEED = 0.25;
    public static double HOLD_SPEED = 0.1;

    private IntakeState intakeState;
    private double velocity;

    public enum IntakeState {
        OUTTAKE,
        HOLD,
        CUSTOM,
        DISABLED;
        public double getValue() {
            switch (this) {
                case OUTTAKE:
                    return OUTTAKE_SPEED;
                case HOLD:
                    return HOLD_SPEED;
                case CUSTOM:
                case DISABLED:
                    return 0;
                default:
                    throw new IllegalArgumentException();
            }
        }
    }

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = new MotorEx(hardwareMap, HardwareMapNames.INTAKE_MOTOR, Motor.GoBILDA.RPM_1150);
        intakeMotor.setRunMode(Motor.RunMode.VelocityControl);
        intakeMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setVeloCoefficients(1, 0, 0);
        intakeMotor.setFeedforwardCoefficients(0, 1, 0);
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
