package com.team841.dory.escalator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;

import com.team841.dory.constants.RC;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Escalator Subsystem code. This holds the logic for the escalator.
 */
public class Escalator extends SubsystemBase {

    EscalatorIO io;

    EscalatorIOInputsAutoLogged inputs = new EscalatorIOInputsAutoLogged();

    MotionMagicTorqueCurrentFOC withOutCoralControl = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    MotionMagicTorqueCurrentFOC withCoralControl = new MotionMagicTorqueCurrentFOC(0).withSlot(1);
    DutyCycleOut dutyCycle = new DutyCycleOut(0);

    StatusCode[] latestStatus;

    Position targetPosition = Position.HomeAndIntake;

    public Escalator(EscalatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Escalator", inputs);

        // If in DEV Mode, the robot does more logging.
        if (RC.robotType == RC.RunType.DEV) {
            Logger.recordOutput("Escalator/AtHome", this.atPosition(Position.HomeAndIntake));
            Logger.recordOutput("Escalator/TargetPosition", this.targetPosition.toString());
            Logger.recordOutput("Escalator/PositionRadian", this.inputs.rightMotorPosition.in(Units.Rotation));
            Logger.recordOutput("Escalator/Logical Check",
                    this.targetPosition == Position.HomeAndIntake &&
                            Math.abs(inputs.rightMotorPosition.in(Units.Rotation) - Position.HomeAndIntake.getPosition()) < 2
                            && !(inputs.rightMotorPosition.in(Units.Rotation) < 0.01));
        }

        Logger.recordOutput("Escalator/latencyPeriodicSec", Timer.getTimestamp() - timestamp);
    }

    /**
     * The coral has non-negligible weight, there is two PID loops, one for with coral, and one without
     * @param position target preset location
     * @param hasCoral whether the shooter is using the coral control or not
     */
    public void setPosition(Position position, boolean hasCoral) {
        if (hasCoral) {
            this.latestStatus = io.setControl(withCoralControl.withPosition(position.getPosition()));
        } else {
            this.latestStatus = io.setControl(withOutCoralControl.withPosition(position.getPosition()));
        }

        this.targetPosition = position;
    }

    public void zero() {
        io.resetPositions();
    }

    public boolean deviceStatusOK() {
        return latestStatus[0].isOK() && latestStatus[1].isOK();
    }

    public boolean atPosition(Position position) {
        return Math.abs(inputs.rightMotorPosition.in(Units.Rotation) - position.getPosition()) < 0.5;
    }

    /**
     * Duty cycle holds down elevator command.
     * Because the PID turning is not the best, when intaking, we do a passive hold down
     * @return Command
     */
    public Command passiveHoldDown() {
        return new RunCommand(
                () -> this.io.setControl(this.dutyCycle.withOutput(-0.025)), this
        ).onlyIf(() -> (this.targetPosition == Position.HomeAndIntake
                && Math.abs(inputs.rightMotorPosition.in(Units.Rotation) - Position.HomeAndIntake.getPosition()) < 2
                && !(inputs.rightMotorPosition.in(Units.Rotation) < 0.01))).withName("passiveEscalatorHoldDown");
    }

    public Position getTarget() {
        return this.targetPosition;
    }

    /**
     * Manual escalator go up command
     * @return Command
     */
    public Command goUp() {
        return new RunCommand(
                () -> this.io.setControl(this.dutyCycle.withOutput(0.1)), this)
                .withName("EscalatorGoUp")
                .finallyDo(() -> this.io.setControl(this.dutyCycle.withOutput(0)));
    }

    /**
     * Manual escalator go down command
     * @return Command
     */
    public Command goDown() {
        return new RunCommand(() -> this.io.setControl(this.dutyCycle.withOutput(-0.1)), this).withName("EscalatorGoDown").finallyDo(() -> this.io.setControl(this.dutyCycle.withOutput(0)));
    }

    /**
     * An enum that stores the positions of all the scoring locations. This way we can set the elevator to go to L1
     * with words as opposed to just pure numbers.
     */
    public enum Position {
        HomeAndIntake(0),
        L1(1),
        L2(5.118 - 0.26123),
        L3(11.5463 - 0.26123),
        L4(22.0844 - 0.26123 + 0.45),
        Hold(7.0), Other(-1);

        private final double position;

        // Constructor
        Position(double position) {
            this.position = position;
        }

        // Getter method
        public double getPosition() {
            return position;
        }
    }
}
