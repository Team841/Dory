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

    Position targetPosition;
    
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

        this.targetPosition = position;
    }

    public void zero(){
        io.resetPositions();
    }

    public boolean deviceStatusOK(){
        return latestStatus[0].isOK() && latestStatus[1].isOK();
    }

    public boolean atPosition(Position position){
        return Math.abs(inputs.rightMotorPosition.magnitude() - position.getPosition()) < 0.1;
    }

    public Position getTarget(){
        return this.targetPosition;
    }

    public enum Position {
        HomeAndIntake(0),
        L1(1),
        L2(5.118 - 0.26123),
        L3(11.5463 - 0.26123),
        L4(22.0844 - 0.26123),
        Hold(2.0),
        Other(-1);

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
