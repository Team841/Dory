package com.team841.dory.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the shooter, following the Advantage Kit format.
 */
public interface ShooterIO {

    // A lot of logging data is turned off to improve loop times.
    @AutoLog
    public static class ShooterIOInputs {
//        public MeasurementHealthValue frontCANrangeHealth;
//        public MeasurementHealthValue backCANrangeHealth;
//        public Time frontCANrangeMeasurementTime;
//        public Time backCANrangeMeasurementTime;
//        public double frontCANrangeSignalStrength;
//        public double backCANrangeSignalStrength;
        public Distance frontCANrangeDistance;
        public Distance backCANrangeDistance;
//        public Distance frontCANrangeStandardDeviation;
//        public Distance backCANrangeStandardDeviation;
//        public double frontCANrangeAmbientSignal;
//        public double backCANrangeAmbientSignal;
//        public boolean frontCANrangeIsDetected;
//        public boolean backCANrangeIsDetected;

        public AngularVelocity KrakenVelocity;
        public AngularAcceleration KrakenAcceleration;
        public double KrakenDutyCycleOut;
    }

    public void updateInputs(ShooterIOInputs inputs);

    public StatusCode setControl(DutyCycleOut control);

    public void stopMotor();
}
