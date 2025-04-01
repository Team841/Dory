package com.team841.dory.escalator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.SC;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class EscalatorIOKraken implements EscalatorIO {

    TalonFX leftMotor = new TalonFX(SC.Escalator.left, RC.canivoreCANBus);
    TalonFX rightMotor = new TalonFX(SC.Escalator.right, RC.canivoreCANBus);

    private final StatusSignal<Angle> rightPosition;
    private final StatusSignal<Angle> leftPosition;

    private final StatusSignal<AngularVelocity> rightVelocity;
    private final StatusSignal<AngularVelocity> leftVelocity;

    private final StatusSignal<AngularAcceleration> rightAcceleration;
    private final StatusSignal<AngularAcceleration> leftAcceleration;

    private final StatusSignal<Current> rightTorqueCurrent;
    private final StatusSignal<Current> leftTorqueCurrent;

    private final StatusSignal<Double> leftMotorDutyCycleOut;
    private final StatusSignal<Double> rightMotorDutyCycleOut;
    private final StatusSignal<ControlModeValue> controlMode;

    Follower leftFollower = new Follower(SC.Escalator.right, true);

    public EscalatorIOKraken() {
        this.leftMotor.getConfigurator().apply(SC.Escalator.leftConfigs);
        this.rightMotor.getConfigurator().apply(SC.Escalator.rightConfigs);

        resetPositions();

        this.rightPosition = this.rightMotor.getPosition();
        this.leftPosition = this.leftMotor.getPosition();

        this.rightVelocity = this.rightMotor.getVelocity();
        this.leftVelocity = this.leftMotor.getVelocity();

        this.rightAcceleration = this.rightMotor.getAcceleration();
        this.leftAcceleration = this.leftMotor.getAcceleration();

        this.rightTorqueCurrent = this.rightMotor.getTorqueCurrent();
        this.leftTorqueCurrent = this.leftMotor.getTorqueCurrent();

        this.leftMotorDutyCycleOut = this.leftMotor.getDutyCycle();
        this.rightMotorDutyCycleOut = this.rightMotor.getDutyCycle();

        this.controlMode = this.rightMotor.getControlMode();

        BaseStatusSignal.setUpdateFrequencyForAll(100, this.rightPosition, this.leftPosition, this.rightVelocity, this.leftVelocity, this.rightAcceleration, this.leftAcceleration, this.rightTorqueCurrent, this.leftTorqueCurrent, this.rightMotorDutyCycleOut, this.leftMotorDutyCycleOut
        );

        this.leftMotor.setControl(leftFollower);
    }

    @Override
    public void updateInputs(EscalatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(this.rightPosition, this.leftPosition, this.rightVelocity, this.leftVelocity, this.rightAcceleration, this.leftAcceleration, this.rightTorqueCurrent, this.leftTorqueCurrent, this.rightMotorDutyCycleOut, this.leftMotorDutyCycleOut);

        inputs.leftMotorPosition = this.leftPosition.getValue();
        inputs.leftMotorVelocity = this.leftVelocity.getValue();
        inputs.leftMotorAcceleration = this.leftAcceleration.getValue();
        inputs.leftMotorTorqueCurrent = this.leftTorqueCurrent.getValue();

        inputs.rightMotorPosition = this.rightPosition.getValue();
        inputs.rightMotorVelocity = this.rightVelocity.getValue();
        inputs.rightMotorAcceleration = this.rightAcceleration.getValue();
        inputs.rightMotorTorqueCurrent = this.rightTorqueCurrent.getValue();

        inputs.leftMotorDutyCycleOut = this.leftMotorDutyCycleOut.getValue();
        inputs.rightMotorDutyCycleOut = this.rightMotorDutyCycleOut.getValue();

        inputs.controlMode = this.controlMode.getValue();
    }

    @Override
    public void resetPositions() {
        this.resetPositions(0.0);
    }

    @Override
    public void resetPositions(double position) {
        this.resetPositions(position, position);
    }

    @Override
    public void resetPositions(double leftPosition, double rightPosition) {
        this.leftMotor.setPosition(leftPosition);
        this.rightMotor.setPosition(rightPosition);
    }

    @Override
    public StatusCode[] setControl(ControlRequest control) {
        return new StatusCode[]{this.rightMotor.setControl(control), this.leftMotor.setControl(leftFollower)};
    }
}
