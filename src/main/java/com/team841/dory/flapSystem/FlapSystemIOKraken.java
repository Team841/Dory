package com.team841.dory.flapSystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.team841.dory.constants.SC;

import edu.wpi.first.units.measure.*;

/**
 * Physical IO implementation for the flap system, including intake, flap, and hang motors.
 */
public class FlapSystemIOKraken implements FlapSystemIO {
    public TalonFX intakeMotor = new TalonFX(SC.flapSystem.intakeMotor, "rio");
    public TalonFX flapMotor = new TalonFX(SC.flapSystem.flapMotor, "rio");
    public CANrange canrange = new CANrange(SC.flapSystem.intakeCanRangeId, "rio");
    public TalonFX hangMotor = new TalonFX(SC.flapSystem.hangMotor, "rio");
    public TalonFX hangMotor2 = new TalonFX(SC.flapSystem.hangMotor2, "rio");

    // We use status signals to get the values that we want to log.
//    StatusSignal<MeasurementHealthValue> CANrangeHealth;
//    StatusSignal<Time> CANrangeMeasurementTime;
//    StatusSignal<Double> CANrangeSignalStrength;
    StatusSignal<Distance> CANrangeDistance;
//    StatusSignal<Distance> CANrangeStandardDeviation;
//    StatusSignal<Double> CANrangeAmbientSignal;
//    StatusSignal<Boolean> CANrangeIsDetected;

    StatusSignal<AngularVelocity> IntakeVelocity;
    StatusSignal<AngularAcceleration> IntakeAcceleration;
    StatusSignal<Double> IntakeDutyCycleOut;

    StatusSignal<AngularVelocity> FlapVelocity;
    StatusSignal<AngularAcceleration> FlapAcceleration;
    StatusSignal<Angle> FlapPosition;
    StatusSignal<Double> FlapDutyCycleOut;

    StatusSignal<AngularVelocity> HangVelocity;
    StatusSignal<AngularAcceleration> HangAcceleration;
    StatusSignal<Angle> HangPosition;
    StatusSignal<Double> HangDutyCycleOut;

    StatusSignal<AngularVelocity> Hang2Velocity;
    StatusSignal<AngularAcceleration> Hang2Acceleration;
    StatusSignal<Angle> Hang2Position;
    StatusSignal<Double> Hang2DutyCycleOut;

    public FlapSystemIOKraken() {
        this.intakeMotor.getConfigurator().apply(SC.flapSystem.configs);
        this.intakeMotor.setNeutralMode(NeutralModeValue.Brake);

        this.flapMotor.getConfigurator().apply(
                SC.flapSystem.configs.withCurrentLimits(
                        new CurrentLimitsConfigs()
                                .withSupplyCurrentLimit(5)
                                .withSupplyCurrentLimitEnable(true)));
        this.flapMotor.setNeutralMode(NeutralModeValue.Brake);

        this.canrange.getConfigurator().apply(SC.flapSystem.CanrangeConfigs);

        this.hangMotor.getConfigurator().apply(SC.flapSystem.configs);
        this.hangMotor.setNeutralMode(NeutralModeValue.Brake);

        this.hangMotor2.getConfigurator().apply(SC.flapSystem.configs);
        this.hangMotor2.setNeutralMode(NeutralModeValue.Brake);

//        this.CANrangeHealth = this.canrange.getMeasurementHealth();
//        this.CANrangeMeasurementTime = this.canrange.getMeasurementTime();
//        this.CANrangeSignalStrength = this.canrange.getSignalStrength();
        this.CANrangeDistance = this.canrange.getDistance();
//        this.CANrangeStandardDeviation = this.canrange.getDistanceStdDev();
//        this.CANrangeAmbientSignal = this.canrange.getAmbientSignal();
//        this.CANrangeIsDetected = this.canrange.getIsDetected();

        this.IntakeVelocity = this.intakeMotor.getVelocity();
        this.IntakeAcceleration = this.intakeMotor.getAcceleration();
        this.IntakeDutyCycleOut = this.intakeMotor.getDutyCycle();

        this.FlapVelocity = this.flapMotor.getVelocity();
        this.FlapAcceleration = this.flapMotor.getAcceleration();
        this.FlapPosition = this.flapMotor.getPosition();
        this.FlapDutyCycleOut = this.flapMotor.getDutyCycle();

        this.HangVelocity = this.hangMotor.getVelocity();
        this.HangAcceleration = this.hangMotor.getAcceleration();
        this.HangPosition = this.hangMotor.getPosition();
        this.HangDutyCycleOut = this.hangMotor.getDutyCycle();

        this.Hang2Velocity = this.hangMotor2.getVelocity();
        this.Hang2Acceleration = this.hangMotor2.getAcceleration();
        this.Hang2Position = this.hangMotor2.getPosition();
        this.Hang2DutyCycleOut = this.hangMotor2.getDutyCycle();

        BaseStatusSignal.setUpdateFrequencyForAll(50,
//                this.CANrangeHealth, this.CANrangeMeasurementTime,
//                this.CANrangeSignalStrength, this.CANrangeDistance,
//                this.CANrangeStandardDeviation, this.CANrangeAmbientSignal,
//                this.CANrangeIsDetected,
                this.IntakeVelocity, this.IntakeAcceleration, this.IntakeDutyCycleOut, this.FlapVelocity, this.FlapAcceleration, this.FlapPosition, this.FlapDutyCycleOut, this.HangVelocity, this.HangAcceleration, this.HangPosition, this.HangDutyCycleOut, this.Hang2Velocity, this.Hang2Acceleration, this.Hang2Position, this.Hang2DutyCycleOut);
        this.hangMotor2.setControl(new Follower(SC.flapSystem.hangMotor, false));
    }

    @Override
    public void updateInputs(FlapSystemIOInputs inputs) {
        // Update the status signals, so that we can log the latest values.
        BaseStatusSignal.refreshAll(
//                this.CANrangeHealth, this.CANrangeMeasurementTime,
//                this.CANrangeSignalStrength, this.CANrangeDistance,
//                this.CANrangeStandardDeviation, this.CANrangeAmbientSignal,
//                this.CANrangeIsDetected,
                this.IntakeVelocity, this.IntakeAcceleration,
                this.IntakeDutyCycleOut, this.FlapVelocity,
                this.FlapAcceleration, this.FlapPosition,
                this.FlapDutyCycleOut, this.HangVelocity,
                this.HangAcceleration, this.HangPosition,
                this.HangDutyCycleOut, this.Hang2Velocity,
                this.Hang2Acceleration, this.Hang2Position,
                this.Hang2DutyCycleOut);

//        inputs.CANrangeHealth = this.CANrangeHealth.getValue();
//        inputs.CANrangeMeasurementTime = this.CANrangeMeasurementTime.getValue();
//        inputs.CANrangeSignalStrength = this.CANrangeSignalStrength.getValue();
        inputs.CANrangeDistance = this.CANrangeDistance.getValue();
//        inputs.CANrangeStandardDeviation = this.CANrangeStandardDeviation.getValue();
//        inputs.CANrangeAmbientSignal = this.CANrangeAmbientSignal.getValue();
//        inputs.CANrangeIsDetected = this.CANrangeIsDetected.getValue();

        inputs.IntakeVelocity = this.IntakeVelocity.getValue();
        inputs.IntakeAcceleration = this.IntakeAcceleration.getValue();
        inputs.IntakeDutyCycleOut = this.IntakeDutyCycleOut.getValue();

        inputs.FlapVelocity = this.FlapVelocity.getValue();
        inputs.FlapAcceleration = this.FlapAcceleration.getValue();
        inputs.FlapPosition = this.FlapPosition.getValue();
        inputs.FlapDutyCycleOut = this.FlapDutyCycleOut.getValue();

        inputs.HangVelocity = this.HangVelocity.getValue();
        inputs.HangAcceleration = this.HangAcceleration.getValue();
        inputs.HangPosition = this.HangPosition.getValue();
        inputs.HangDutyCycleOut = this.HangDutyCycleOut.getValue();

        inputs.Hang2Velocity = this.Hang2Velocity.getValue();
        inputs.Hang2Acceleration = this.Hang2Acceleration.getValue();
        inputs.Hang2Position = this.Hang2Position.getValue();
        inputs.Hang2DutyCycleOut = this.Hang2DutyCycleOut.getValue();
    }

    @Override
    public StatusCode setControlIntake(DutyCycleOut control) {
        return this.intakeMotor.setControl(control);
    }

    @Override
    public StatusCode setControlFlapper(DutyCycleOut control) {
        return this.flapMotor.setControl(control);
    }

    @Override
    public StatusCode setControlHang(DutyCycleOut control) {
        return this.hangMotor.setControl(control);
    }

    @Override
    public void stopIntake() {
        this.intakeMotor.stopMotor();
    }

    @Override
    public void stopFlapper() {
        this.flapMotor.stopMotor();
    }

    @Override
    public void stopHang() {
        this.hangMotor.stopMotor();
    }

}
