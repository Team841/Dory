package com.team841.dory.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    DutyCycleOut dutyCycle;

    StatusCode latestStatusCode;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        Logger.recordOutput("Shooter/latencyPeriodicSec", Timer.getTimestamp() - timestamp);
    }

    public boolean BackSensorTriggerd(){
        return inputs.backCANrangeIsDetected;
    }

    public boolean frontSensorTriggerd(){
        return inputs.frontCANrangeIsDetected;
    }

    public void setDutyCycle(double output) {
        this.latestStatusCode = io.setControl(dutyCycle.withOutput(output));
    }

    public void setDutyCycle(ShooterSpeeds speed){
       this.latestStatusCode =  io.setControl(dutyCycle.withOutput(speed.getDutyCyle()));
    }

    public boolean deviceStatusOK(){
        return latestStatusCode.isOK();
    }

    public enum ShooterSpeeds {
        Intake(0.5),
        Stopped(0),
        ShootL2AndL3(0.5),
        ShootL4(0.75),
        ShooterL1(0.25);

        private final double dutyCyle;

        ShooterSpeeds(double dutyCyle) {
            this.dutyCyle = dutyCyle;
        }

        public double getDutyCyle() {
            return dutyCyle;
        }
    }
}
