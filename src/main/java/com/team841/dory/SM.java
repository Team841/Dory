package com.team841.dory;

import com.team841.dory.drive.Drivetrain;
import com.team841.dory.escalator.Escalator;
import com.team841.dory.shooter.Shooter;
import org.littletonrobotics.junction.AutoLogOutput;

public class SM {

    private Drivetrain drivetrain;
    private Escalator escalator;
    private Shooter shooter;

    @AutoLogOutput
    private AutomationState CurrentAutomationState;

    @AutoLogOutput
    private AutomationState requestAutomationState;

    @AutoLogOutput
    private SuperstructureStates currentSuperstructureStates;

    @AutoLogOutput
    private SuperstructureStates requestedSuperstructureStates;

    @AutoLogOutput
    private SuperstructureStates previousSuperstructureStates;

    @AutoLogOutput
    private ShooterState currentShooterState;

    @AutoLogOutput
    private boolean needsToShoot = false;

    public SM(Drivetrain drivetrain, Escalator escalator, Shooter shooter) {
        this.drivetrain = drivetrain;
        this.escalator = escalator;
        this.shooter = shooter;
    }

    public boolean requestState(SuperstructureStates superstructureStates, AutomationState automationState) {
        try {
            this.requestedSuperstructureStates = superstructureStates;
            this.requestAutomationState = automationState;
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public boolean loop() {
        try{
            if (this.requestedSuperstructureStates != null && this.currentSuperstructureStates != SuperstructureStates.travel) {
                this.previousSuperstructureStates = this.currentSuperstructureStates;
                this.currentSuperstructureStates = requestedSuperstructureStates;
                this.requestedSuperstructureStates = null;
            }

            switch (this.currentSuperstructureStates) {
                case travel -> {
                    this.currentSuperstructureStates = this.escalator.getWhichSuperstructureState();
                    if ((this.escalator.getTarget() == Escalator.Position.L1
                            || this.escalator.getTarget() == Escalator.Position.L2
                            || this.escalator.getTarget() == Escalator.Position.L3) && this.shooter.shooterHasCoral()) {
                        this.needsToShoot = true;
                    }
                }
                case Home -> {
                    this.escalator.setPosition(Escalator.Position.HomeAndIntake, this.shooter.shooterHasCoral());
                }
                case L1 -> {
                    this.escalator.setPosition(Escalator.Position.L1, this.shooter.shooterHasCoral());
                }
                case L2 -> {
                    this.escalator.setPosition(Escalator.Position.L2, this.shooter.shooterHasCoral());
                }
                case L3 -> {
                    this.escalator.setPosition(Escalator.Position.L3, this.shooter.shooterHasCoral());
                }
                case L4 -> {
                    this.escalator.setPosition(Escalator.Position.L4, this.shooter.shooterHasCoral());
                }
            }

            if (this.needsToShoot && this.escalator.atPosition(this.escalator.getTarget())){
                if (this.escalator.)
            }

            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public enum AutomationState {
        Zero,
        Semi,
        Full
    }

    public enum ShooterState {
        stopped,
        intake,
        shooter
    }

    public enum SuperstructureStates {
        travel(Escalator.Position.Other),
        Home(Escalator.Position.Other),
        L1(Escalator.Position.L1),
        L2(Escalator.Position.L2),
        L3(Escalator.Position.L3),
        L4(Escalator.Position.L4);
        private final Escalator.Position position;

        private SuperstructureStates(Escalator.Position position) {
            this.position = position;
        }

        Escalator.Position getPosition() {
            return position;
        }

        public static SuperstructureStates getSuperstructureStates(Escalator.Position position) {
            for (SuperstructureStates state : SuperstructureStates.values()) {
                if (state.getPosition().equals(position)) {
                    return state;
                }
            }

            return null;
        }
    }
}
