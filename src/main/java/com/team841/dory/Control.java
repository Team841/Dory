package com.team841.dory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team841.dory.drive.Commands.AutoSnap;
import com.team841.dory.drive.Drivetrain;
import com.team841.dory.escalator.Escalator;
import com.team841.dory.escalator.MoveCommand;
import com.team841.dory.flapSystem.FlapSystem;
import com.team841.dory.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.*;

public class Control {

    public static double scoreTimeout = 0.3;

    private final Drivetrain drivetrain;
    private final Escalator escalator;
    private final Shooter shooter;
    private final FlapSystem flapSystem;

//    public final SequentialCommandGroup snapScoreLeftL4;
//    public final SequentialCommandGroup snapScoreLeftL3;
//    public final SequentialCommandGroup snapScoreLeftL2;
//
//    public final SequentialCommandGroup snapScoreRightL4;
//    public final SequentialCommandGroup snapScoreRightL3;
//    public final SequentialCommandGroup snapScoreRightL2;

    public final Command snapScoreL4;
    public final Command snapScoreL3;
    public final Command snapScoreL2;

    public final Command noSnapAutoScoreL4;
    public final Command noSnapAutoScoreL3;
    public final Command noSnapAutoScoreL2;

    public final Command escalatorGoHome;

    public final Command intake;

//    public final Command hangRetract;
//    public final Command hangDeploy;
//
//    public final Command intakeUp;
//    public final Command intakeDown;

    public Control(Drivetrain drivetrain, Escalator escalator, Shooter shooter, FlapSystem flapSystem) {
        this.drivetrain = drivetrain;
        this.escalator = escalator;
        this.shooter = shooter;
        this.flapSystem = flapSystem;

        if (!AutoBuilder.isConfigured()) {
            drivetrain.configureAutoBuilder();
        }

        NamedCommands.registerCommand("L2", new SequentialCommandGroup(
                new MoveCommand(this.escalator, Escalator.Position.L2, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear));
        NamedCommands.registerCommand("L4", new SequentialCommandGroup(
                new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("AutoL4", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new AutoSnap(this.drivetrain), new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral, this.shooter::escalatorClear)), this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear)
        );
        NamedCommands.registerCommand("Intake", Intake());
        NamedCommands.registerCommand("PassiveRaise", new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.Hold, false)));
        NamedCommands.registerCommand("FlapDown", new InstantCommand(() -> this.flapSystem.setFlapperDutyCycle(0.25)).withTimeout(0.75).finallyDo(flapSystem::stopFlapper));

//        this.snapScoreLeftL4 =
//                new SequentialCommandGroup(
//                        new Snapping(this.drivetrain, true),
//                        new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral),
//                        this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
//                        new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)));
//        this.snapScoreLeftL3 =
//                new SequentialCommandGroup(
//                        new Snapping(this.drivetrain, true),
//                        new MoveCommand(this.escalator, Escalator.Position.L3, this.shooter::shooterHasCoral),
//                        this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
//                        new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)));
//        this.snapScoreLeftL2 =
//                new SequentialCommandGroup(
//                        new Snapping(this.drivetrain, true),
//                        new MoveCommand(this.escalator, Escalator.Position.L2, this.shooter::shooterHasCoral),
//                        this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout),
//                        new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)));
//
//        this.snapScoreRightL4 =
//                new SequentialCommandGroup(
//                        new Snapping(this.drivetrain, false),
//                        new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral),
//                        this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
//                        new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)));
//        this.snapScoreRightL3 =
//                new SequentialCommandGroup(
//                        new Snapping(this.drivetrain, false),
//                        new MoveCommand(this.escalator, Escalator.Position.L3, this.shooter::shooterHasCoral),
//                        this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
//                        new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)));
//        this.snapScoreRightL2 =
//                new SequentialCommandGroup(
//                        new Snapping(this.drivetrain, false),
//                        new MoveCommand(this.escalator, Escalator.Position.L2, this.shooter::shooterHasCoral),
//                        this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout),
//                        new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)));

        this.snapScoreL4 = new SequentialCommandGroup(
                new AutoSnap(this.drivetrain), new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);

        this.snapScoreL3 = new SequentialCommandGroup(
                new AutoSnap(this.drivetrain), new MoveCommand(this.escalator, Escalator.Position.L3, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);

        this.snapScoreL2 = new SequentialCommandGroup(
                new AutoSnap(this.drivetrain), new MoveCommand(this.escalator, Escalator.Position.L2, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL4 = new SequentialCommandGroup(
                new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);
        this.noSnapAutoScoreL3 = new SequentialCommandGroup(
                new MoveCommand(this.escalator, Escalator.Position.L3, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);
        this.noSnapAutoScoreL2 = new SequentialCommandGroup(
                new MoveCommand(this.escalator, Escalator.Position.L2, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);

        this.intake = Intake();

        this.escalatorGoHome = new MoveCommand(this.escalator, Escalator.Position.HomeAndIntake, this.shooter::shooterHasCoral, this.shooter::escalatorClear).onlyIf(this.shooter::escalatorClear);

//        this.hangRetract = hang.retract();
//        this.hangDeploy = hang.deploy();
//
//        this.intakeUp = hang.intakeUp();
//        this.intakeDown = hang.intakeDown();
    }

    public Command Intake() {
        return new ConditionalCommand(
                new ParallelRaceGroup(
                        this.shooter.runShooterIntake(), this.flapSystem.runIntake()
                ), new SequentialCommandGroup(
                        //                        new MoveCommand(this.escalator, Escalator.Position.HomeAndIntake, this.shooter::shooterHasCoral, this.shooter::escalatorClear),
                        this.escalator.passiveHoldDown().withTimeout(0.1), new ParallelRaceGroup(
                                this.shooter.runShooterIntake(), this.flapSystem.runIntake()
                        )

                ).onlyIf(this.shooter::escalatorClear), () -> this.escalator.atPosition(Escalator.Position.HomeAndIntake)
        );
    }
}
