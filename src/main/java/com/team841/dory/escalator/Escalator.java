package com.team841.dory.escalator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Escalator extends SubsystemBase{

    EscalatorIO io;

    EscalatorIOInputsAutoLogged inputs = new EscalatorIOInputsAutoLogged();

    MotionMagicTorqueCurrentFOC withOutCoralControl = new MotionMagicTorqueCurrentFOC(0).withSlot(0);
    MotionMagicTorqueCurrentFOC withCoralControl = new MotionMagicTorqueCurrentFOC(0).withSlot(1);

    StatusCode[] latestStatus;
    
    public Escalator(EscalatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Escalator", inputs);
        Logger.recordOutput("Escalator/latencyPeriodicSec", Timer.getTimestamp() - timestamp);
    }

    public void setPosition(Position position, boolean hasCoral) {
        if (hasCoral) {
            this.latestStatus = io.setControl(withCoralControl.withPosition(position.getPosition()));
        } else {
            this.latestStatus = io.setControl(withOutCoralControl.withPosition(position.getPosition()));
        }
    }

    public void zero(){
        io.resetPositions();
    }

    public boolean deviceStatusOK(){
        return latestStatus[0].isOK() && latestStatus[1].isOK();
    }

    public enum Position {
        HomeAndIntake(5.9),
        L1(15.25),
        L2(23.57),
        L3(36.02),
        L4(57.95),
        Hold(2.0);

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
