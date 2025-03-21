package com.team841.dory.flapSystem;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.team841.dory.constants.RC;
import com.team841.dory.shooter.Shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FlapSystem extends SubsystemBase {

    FlapSystemIO io;

    FlapSystemIOInputsAutoLogged inputs = new FlapSystemIOInputsAutoLogged();

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    StatusCode latestStatusCode;

    public FlapSystem(FlapSystemIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("FlapSystem", inputs);

        if (RC.robotType == RC.RunType.DEV){}
        Logger.recordOutput("FlapSystem/latencyPeriodicSec", Timer.getTimestamp() - timestamp);
    }

    public void setIntakeDutyCycle(double output){
        this.latestStatusCode = io.setControlIntake(dutyCycle.withOutput(output));
    }

    public void stopIntake(){
        io.stopIntake();
    }

    public Command runIntake(){
        return new RunCommand(
                () -> {
                    this.setIntakeDutyCycle(0.25);
                }
        )
                .withName("runShooterIntakeCommand")
                .finallyDo(this::stopIntake);
    }

    public boolean deviceStatusOK(){
        return latestStatusCode.isOK();
    }
}
