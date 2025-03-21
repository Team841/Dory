package com.team841.dory.shooter;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.team841.dory.constants.RC;
import com.team841.dory.escalator.Escalator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    ShooterIO io;
    ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    StatusCode latestStatusCode;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getTimestamp();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if (RC.robotType == RC.RunType.DEV){
            Logger.recordOutput("Shooter/isclear", this.escalatorClear());
            Logger.recordOutput("Shooter/hasCoral", this.shooterHasCoral());
        }
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
       this.latestStatusCode =  io.setControl(dutyCycle.withOutput(speed.getDutyCycle()));
    }

    public void stop(){
        io.stopMotor();
    }

    public boolean escalatorClear(){
        return (inputs.frontCANrangeDistance.magnitude() < 0.05 && inputs.backCANrangeDistance.magnitude() > 0.05) ||
                (inputs.frontCANrangeDistance.magnitude() > 0.05 && inputs.backCANrangeDistance.magnitude() > 0.05);
    }

//    @AutoLogOutput
    public boolean shooterHasCoral(){
        return (inputs.frontCANrangeDistance.magnitude() < 0.05 && inputs.backCANrangeDistance.magnitude() > 0.05);
    }


    public boolean backSensor(){
        return inputs.backCANrangeDistance.magnitude() < 0.05;
    }
    public boolean deviceStatusOK(){
        return latestStatusCode.isOK();
    }

    public Command runShooterScore(Escalator.Position atPosition, double timout){
        return new RunCommand(
                () -> {
                    if (atPosition == Escalator.Position.L2 || atPosition == Escalator.Position.L3){
                        setDutyCycle(ShooterSpeeds.ShootL2AndL3);
                    } else if (atPosition == Escalator.Position.L4) {
                        setDutyCycle(ShooterSpeeds.ShootL4);
                    } else if (atPosition == Escalator.Position.L1){
                        setDutyCycle(ShooterSpeeds.ShooterL1);
                    }
                }
        )
                .withName("runShooterScoreCommand")
                .withTimeout(timout)
                .finallyDo(this::stop);
    }

    public Command runShooterIntake(){
        return new RunCommand(
                () -> {
                    this.setDutyCycle(ShooterSpeeds.Intake);
                }
        )
                .withName("runShooterIntakeCommand")
                .until(this::shooterHasCoral)
                .finallyDo(this::stop);
//                .andThen(new RunCommand(() -> this.setDutyCycle(ShooterSpeeds.Intake)).withTimeout(0.1))
//                .andThen(this::stop);
    }

    public enum ShooterSpeeds {
        Intake(0.08),
        Stopped(0),
        ShootL2AndL3(0.2),
        ShootL4(0.75),
        ShooterL1(0.25);

        private final double dutyCycle;

        ShooterSpeeds(double dutyCycle) {
            this.dutyCycle = dutyCycle;
        }

        public double getDutyCycle() {
            return dutyCycle;
        }
    }
}
