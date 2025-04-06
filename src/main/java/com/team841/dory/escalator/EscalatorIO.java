package com.team841.dory.escalator;

import edu.wpi.first.units.measure.*;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.ControlModeValue;

/**
 * IO interface for the elevator, following the Advantage Kit format.
 */
public interface EscalatorIO {

    @AutoLog
    public static class EscalatorIOInputs {
        public Angle leftMotorPosition;
        public AngularVelocity leftMotorVelocity;
        public AngularAcceleration leftMotorAcceleration;
        public Current leftMotorTorqueCurrent;
        public Angle rightMotorPosition;
        public AngularVelocity rightMotorVelocity;
        public AngularAcceleration rightMotorAcceleration;
        public Current rightMotorTorqueCurrent;

        public double rightMotorDutyCycleOut;
        public double leftMotorDutyCycleOut;
        public ControlModeValue controlMode;
    }

    public void updateInputs(EscalatorIOInputs inputs);

    public void resetPositions();

    public void resetPositions(double position);

    public void resetPositions(double leftPosition, double rightPosition);

    public StatusCode[] setControl(ControlRequest control);
}
