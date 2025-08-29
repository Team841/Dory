package com.team841.dory.lights;

import com.team841.dory.Robot;
import com.team841.dory.shooter.Shooter;
import com.team841.dory.flapSystem.FlapSystem;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.nio.channels.Pipe;

import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {

    private final LEDIO io;
    private final LedIOInputsAutoLogged inputs = new LedIOInputsAutoLogged();

    private Shooter shooter;
    private FlapSystem flapSystem;
    private Timer timer;

    private int endgameMsFlash;
    private int endgameTime;

    /** Creates a new LED. */
    public LED(LEDIO io, Shooter shooter, FlapSystem flapSystem) {
       this.io = io;
       this.shooter = shooter;
       this.flapSystem = flapSystem;
       this.endgameMsFlash = 0;
       this.endgameTime = 20;
       this.timer = new Timer();
       this.timer.start();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("LED", inputs);

        if (DriverStation.getMatchTime() < (this.endgameTime + 1) && DriverStation.getMatchTime() > this.endgameTime) {
            this.endgameMsFlash = 0;
        }

        if (DriverStation.getMatchTime() < this.endgameTime && this.endgameMsFlash < 100 && DriverStation.isEnabled() && DriverStation.isAutonomous() == false) {
            if (this.timer.hasElapsed(0.2)) {
                io.setColor("blue");
                this.timer.restart();
            } else if (this.timer.hasElapsed(0.1)) {
                io.setColor("off");
            }
            this.endgameMsFlash++;
        } else if (shooter.shooterHasCoral()) {
            io.setColor("green");
        } else if (shooter.escalatorClear() == false){
            if (this.timer.hasElapsed(0.2)) {
                io.setColor("red");
                this.timer.restart();
            } else if (this.timer.hasElapsed(0.1)) {
                io.setColor("off");
            }
        } else {
            if (DriverStation.getMatchTime() < this.endgameTime && DriverStation.isEnabled() && DriverStation.isAutonomous() == false) {
                io.setColor("blue");
            } else {
                io.setColor("yellow");
            }
        }
    }
}