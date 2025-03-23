package com.team841.dory.flapSystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MeasurementHealthValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team841.dory.constants.SC;
import edu.wpi.first.units.measure.*;

public class FlapSystemIOKraken implements FlapSystemIO{
    public TalonFX intakeMotor = new TalonFX(SC.flapSystem.intakeMotor, "rio");
    public TalonFX flapMotor = new TalonFX(SC.flapSystem.flapMotor, "rio");
    public CANrange canrange = new CANrange(SC.flapSystem.canrange, "rio");

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

    public FlapSystemIOKraken(){
        this.intakeMotor.getConfigurator().apply(SC.flapSystem.configs);
        this.intakeMotor.setNeutralMode(NeutralModeValue.Brake);
        this.flapMotor.getConfigurator().apply(SC.flapSystem.configs);
        this.flapMotor.setNeutralMode(NeutralModeValue.Brake);
        this.canrange.getConfigurator().apply(SC.flapSystem.CanrangeConfigs);

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

        BaseStatusSignal.setUpdateFrequencyForAll(50,
//                this.CANrangeHealth, this.CANrangeMeasurementTime,
//                this.CANrangeSignalStrength, this.CANrangeDistance,
//                this.CANrangeStandardDeviation, this.CANrangeAmbientSignal,
//                this.CANrangeIsDetected,
                this.IntakeVelocity, this.IntakeAcceleration,
                this.IntakeDutyCycleOut,
                this.FlapVelocity, this.FlapAcceleration,
                this.FlapPosition, this.FlapDutyCycleOut);
    }

    @Override
    public void updateInputs(FlapSystemIOInputs inputs) {
        BaseStatusSignal.refreshAll(
//                this.CANrangeHealth, this.CANrangeMeasurementTime,
//                this.CANrangeSignalStrength, this.CANrangeDistance,
//                this.CANrangeStandardDeviation, this.CANrangeAmbientSignal,
//                this.CANrangeIsDetected,
                this.IntakeVelocity, this.IntakeAcceleration,
                this.IntakeDutyCycleOut,
                this.FlapVelocity, this.FlapAcceleration,
                this.FlapPosition, this.FlapDutyCycleOut);

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
    }

    @Override
    public StatusCode setControlIntake(DutyCycleOut control) {
        return this.intakeMotor.setControl(control);
    }
    public StatusCode setControlFlapper(DutyCycleOut control){
        return this.flapMotor.setControl(control);
    }

    @Override
    public void stopIntake() {
        this.intakeMotor.stopMotor();
    }
    public void stopFlapper(){
        this.flapMotor.stopMotor();
    }
}
