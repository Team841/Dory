package com.team841.dory.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs{
        public MeasurementHealthValue frontCANrangeHealth;
        public MeasurementHealthValue backCANrangeHealth;
        public Time frontCANrangeMeasurementTime;
        public Time backCANrangeMeasurementTime;
        public double frontCANrangeSignalStrength;
        public double backCANrangeSignalStrength;
        public Distance frontCANrangeDistance;
        public Distance backCANrangeDistance;
        public Distance frontCANrangeStandardDeviation;
        public Distance backCANrangeStandardDeviation;
        public double frontCANrangeAmbientSignal;
        public double backCANrangeAmbientSignal;
        public boolean frontCANrangeIsDetected;
        public boolean backCANrangeIsDetected;

        public AngularVelocity KrakenVelocity;
        public AngularAcceleration KrakenAcceleration;
        public double KrakenDutyCycleOut;
    }

    public void updateInputs(ShooterIOInputs inputs);

    public StatusCode setControl(DutyCycleOut control);

    public void stopMotor();
}
