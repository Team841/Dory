package com.team841.dory.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.team841.dory.constants.SC;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class ShooterIOKraken implements ShooterIO {

    TalonFX motor;
    CANrange FrontCANrange;
    CANrange BackCANrange;

    StatusSignal<MeasurementHealthValue> frontCANrangeHealth;
    StatusSignal<MeasurementHealthValue> backCANrangeHealth;
    StatusSignal<Time> frontCANrangeMeasurementTime;
    StatusSignal<Time> backCANrangeMeasurementTime;
    StatusSignal<Double> frontCANrangeSignalStrength;
    StatusSignal<Double> backCANrangeSignalStrength;
    StatusSignal<Distance> frontCANrangeDistance;
    StatusSignal<Distance> backCANrangeDistance;
    StatusSignal<Distance> frontCANrangeStandardDeviation;
    StatusSignal<Distance> backCANrangeStandardDeviation;
    StatusSignal<Double> frontCANrangeAmbientSignal;
    StatusSignal<Double> backCANrangeAmbientSignal;
    StatusSignal<Boolean> frontCANrangeIsDetected;
    StatusSignal<Boolean> backCANrangeIsDetected;

    StatusSignal<AngularVelocity> KrakenVelocity;
    StatusSignal<AngularAcceleration> KrakenAcceleration;
    StatusSignal<Double> KrakenDutyCyleOut;

    public ShooterIOKraken() {
        this.motor = new TalonFX(SC.Shooter.MotorId, "rio");
        this.FrontCANrange = new CANrange(SC.Shooter.CanRangeIdFront, "rio");
        this.BackCANrange = new CANrange(SC.Shooter.CanRangeIdBack, "rio");

        this.motor.getConfigurator().apply(SC.Shooter.configs);
        this.FrontCANrange.getConfigurator().apply(SC.Shooter.CanrangeConfigs);
        this.BackCANrange.getConfigurator().apply(SC.Shooter.CanrangeConfigs);

        this.frontCANrangeHealth = this.FrontCANrange.getMeasurementHealth();
        this.backCANrangeHealth = this.BackCANrange.getMeasurementHealth();
        this.frontCANrangeMeasurementTime = this.FrontCANrange.getMeasurementTime();
        this.backCANrangeMeasurementTime = this.BackCANrange.getMeasurementTime();
        this.frontCANrangeSignalStrength = this.FrontCANrange.getSignalStrength();
        this.backCANrangeSignalStrength = this.BackCANrange.getSignalStrength();
        this.frontCANrangeDistance = this.FrontCANrange.getDistance();
        this.backCANrangeDistance = this.BackCANrange.getDistance();
        this.frontCANrangeStandardDeviation = this.FrontCANrange.getDistanceStdDev();
        this.backCANrangeStandardDeviation = this.BackCANrange.getDistanceStdDev();
        this.frontCANrangeAmbientSignal = this.FrontCANrange.getAmbientSignal();
        this.backCANrangeAmbientSignal = this.BackCANrange.getAmbientSignal();
        this.frontCANrangeIsDetected = this.FrontCANrange.getIsDetected();
        this.backCANrangeIsDetected = this.BackCANrange.getIsDetected();

        this.KrakenVelocity = this.motor.getVelocity();
        this.KrakenAcceleration = this.motor.getAcceleration();
        this.KrakenDutyCyleOut = this.motor.getDutyCycle();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
            this.frontCANrangeHealth, this.backCANrangeHealth,
            this.frontCANrangeMeasurementTime, this.backCANrangeMeasurementTime,
            this.frontCANrangeSignalStrength, this.backCANrangeSignalStrength,
            this.frontCANrangeDistance, this.backCANrangeDistance,
            this.frontCANrangeStandardDeviation, this.backCANrangeStandardDeviation,
            this.frontCANrangeAmbientSignal, this.backCANrangeAmbientSignal,
            this.frontCANrangeIsDetected, this.backCANrangeIsDetected,
            this.KrakenVelocity, this.KrakenAcceleration,
            this.KrakenDutyCyleOut
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(this.frontCANrangeHealth, this.backCANrangeHealth,
                this.frontCANrangeMeasurementTime, this.backCANrangeMeasurementTime,
                this.frontCANrangeSignalStrength, this.backCANrangeSignalStrength,
                this.frontCANrangeDistance, this.backCANrangeDistance,
                this.frontCANrangeStandardDeviation, this.backCANrangeStandardDeviation,
                this.frontCANrangeAmbientSignal, this.backCANrangeAmbientSignal,
                this.frontCANrangeIsDetected, this.backCANrangeIsDetected,
                this.KrakenVelocity, this.KrakenAcceleration,
                this.KrakenDutyCyleOut);

        inputs.frontCANrangeHealth = this.frontCANrangeHealth.getValue();
        inputs.backCANrangeHealth = this.backCANrangeHealth.getValue();
        inputs.frontCANrangeMeasurementTime = this.frontCANrangeMeasurementTime.getValue();
        inputs.backCANrangeMeasurementTime = this.backCANrangeMeasurementTime.getValue();
        inputs.frontCANrangeSignalStrength = this.frontCANrangeSignalStrength.getValue();
        inputs.backCANrangeSignalStrength = this.backCANrangeSignalStrength.getValue();
        inputs.frontCANrangeDistance = this.frontCANrangeDistance.getValue();
        inputs.backCANrangeDistance = this.backCANrangeDistance.getValue();
        inputs.frontCANrangeStandardDeviation = this.frontCANrangeStandardDeviation.getValue();
        inputs.backCANrangeStandardDeviation = this.backCANrangeStandardDeviation.getValue();
        inputs.frontCANrangeAmbientSignal = this.frontCANrangeAmbientSignal.getValue();
        inputs.backCANrangeAmbientSignal = this.backCANrangeAmbientSignal.getValue();
        inputs.frontCANrangeIsDetected = this.frontCANrangeIsDetected.getValue();
        inputs.backCANrangeIsDetected = this.backCANrangeIsDetected.getValue();

        inputs.KrakenVelocity = this.KrakenVelocity.getValue();
        inputs.KrakenAcceleration = this.KrakenAcceleration.getValue();
        inputs.KrakenDutyCycleOut = this.KrakenDutyCyleOut.getValue();
    }

    @Override
    public StatusCode setControl(DutyCycleOut control) {
        return this.motor.setControl(control);
    }

    @Override
    public void stopMotor() {
        this.motor.stopMotor();
    }
}
