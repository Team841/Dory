package com.team841.dory;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.Drivetrain;
import com.team841.dory.escalator.Escalator;
import com.team841.dory.escalator.MoveCommand;
import com.team841.dory.escalator.Escalator.Position;
import com.team841.dory.flapSystem.FlapSystem;
import com.team841.dory.shooter.Shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Class that creates all the commands at the start of robot code, allowing for better loop times as the command is not
 * created every time it is need.
 */
public class Control {

    public static double scoreTimeout = 0.5;

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

    /**
     * Class that creates all the commands at initialization which is created at the start of robot code.
     * @param drivetrain drivetrain
     * @param escalator escalator
     * @param shooter shooter
     * @param flapSystem flapSystem
     */
    public Control(Drivetrain drivetrain, Escalator escalator, Shooter shooter, FlapSystem flapSystem) {
        this.drivetrain = drivetrain;
        this.escalator = escalator;
        this.shooter = shooter;
        this.flapSystem = flapSystem;

        if (!AutoBuilder.isConfigured()) {
            drivetrain.configureAutoBuilder();
        }

        NamedCommands.registerCommand("L2", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));
      
        NamedCommands.registerCommand("L3", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L3, 
                        this.shooter::shooterHasCoral, 
                        this.shooter::escalatorClear), 
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout), 
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("L4", new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator, Escalator.Position.L4,
                        this.shooter::shooterHasCoral, this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear));

        NamedCommands.registerCommand("AutoL4", new SequentialCommandGroup(
                new ParallelCommandGroup(
                        AutoSnapInline(),
                        new MoveCommand(
                                this.escalator,
                                Escalator.Position.L4,
                                this.shooter::shooterHasCoral,
                                this.shooter::escalatorClear)),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))
                        .withTimeout(1))
                .onlyIf(this.shooter::escalatorClear)
        );
        NamedCommands.registerCommand("GoDown",
                new InstantCommand(() -> this.escalator.setPosition(Position.HomeAndIntake, false), escalator));

        NamedCommands.registerCommand("Intake", Intake());

        NamedCommands.registerCommand("PassiveRaise",
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.Hold, true)));

        NamedCommands.registerCommand("FlapDown",
                new RunCommand(() -> this.flapSystem.setFlapperDutyCycle(-0.5), flapSystem)
                        .withTimeout(1)
                        .finallyDo(flapSystem::stopFlapper));

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
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L4,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.snapScoreL3 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L3,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.snapScoreL2 = new SequentialCommandGroup(
                AutoSnapInline(),
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(
                        Escalator.Position.L2, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL4 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L4,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(
                        Escalator.Position.L4, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL3 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L3,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL2 = new SequentialCommandGroup(
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.L2,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear),
                this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout),
                new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)))
                .onlyIf(this.shooter::escalatorClear);

        this.intake = Intake();

        this.escalatorGoHome =
                new MoveCommand(
                        this.escalator,
                        Escalator.Position.HomeAndIntake,
                        this.shooter::shooterHasCoral,
                        this.shooter::escalatorClear)
                        .onlyIf(this.shooter::escalatorClear);

//        this.hangRetract = hang.retract();
//        this.hangDeploy = hang.deploy();
//
//        this.intakeUp = hang.intakeUp();
//        this.intakeDown = hang.intakeDown();
    }

    /**
     * Sequences intaking that runs shooter, passive holds down the elevator, and stops when we have a coral.
     * @return Command
     */
    public Command Intake() {
        return new ConditionalCommand(
                new ParallelRaceGroup(
                        this.shooter.runShooterIntake(), this.flapSystem.runIntake()
                ), new SequentialCommandGroup(
                        this.escalator.passiveHoldDown().withTimeout(0.2), new ParallelRaceGroup(
                                this.shooter.runShooterIntake(), this.flapSystem.runIntake()
                        )

                ).onlyIf(this.shooter::escalatorClear), // Make sure that the elevator is clear before running.
                // move elevator down if not at intaking position
                () -> this.escalator.atPosition(Escalator.Position.HomeAndIntake)
        );
    }

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /**
     * Pose to Pose autoAlign command that will drive to the closest scoring command.
     * It is two ProfiledPIDControllers, one for X and one for Y, that will close the robot pose to the target pose
     * We use SwerveRequest.FieldCentricFacingAngle so that we do not have to deal with rotation
     * First we wait for the headingController to point the robot at the right direction before moving.
     * This deals with issues of profiling turning with vx and vy which is not a simple control.
     * @return AutoAlign Command
     */
    public Command AutoSnapInline(){
        ProfiledPIDController vx = drivetrain.vxController;
        ProfiledPIDController vy = drivetrain.vyController;
        AtomicReference<Pose2d> target = new AtomicReference<>();

        return Commands.runOnce(
                // Sets everything up, reset the controllers because we have a non-zero kI
                () -> {
                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                            this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
                    target.set(drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar()));
                    Logger.recordOutput("AutoSnap/PoseTarget", target.get());
                    Translation2d translation2d = this.drivetrain.getPose().getTranslation();
                    vx.reset(translation2d.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
                    vy.reset(translation2d.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
                }
        ).andThen(
                Commands.run(
                        // Points the drivetrain in the right direction
                        () -> this.drivetrain.setControl(
                                drivetrain.driveHeading.withTargetDirection(target.get().getRotation()))
                ).until(
                        // Wait until we are facing the right direction
                        this.drivetrain.driveHeading.HeadingController::atSetpoint
                ).andThen(
                       Commands.run(
                               () -> {
                                   ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                           this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
                                   Translation2d translation2d = this.drivetrain.getPose().getTranslation();

                                   // Add the current setpoint velocity to the new velocity so that it is smoother.
                                   double outputX =
                                           vx.calculate(translation2d.getX(), target.get().getX()) +
                                                   vx.getSetpoint().velocity;
                                   double outputY =
                                           vy.calculate(translation2d.getY(), target.get().getY()) +
                                                   vy.getSetpoint().velocity;

                                   // If it is not scaled, then each controller can output speeds that
                                   // when added up to the final vector will exceed the maximum real speed of the robot.
                                   outputX *= 0.7;
                                   outputY *= 0.7;

                                   // Do some logging to help with debugging
                                   Logger.recordOutput("AutoSnap/outputX", outputX);
                                   Logger.recordOutput("AutoSnap/outputY", outputY);
                                   Logger.recordOutput("AutoSnap/vxAtGoal", vx.atGoal());
                                   Logger.recordOutput("AutoSnap/vyAtGoal", vy.atGoal());

                                   this.drivetrain.setControl(
                                           this.drivetrain.driveHeading
                                                   .withVelocityY(outputY)
                                                   .withVelocityX(outputX)
                                                   .withTargetDirection(target.get().getRotation())
                                   );
                               }
                       ).until(
                               () -> vx.atGoal() && vy.atGoal()
                       ).finallyDo(
                               () -> this.drivetrain.setControl(
                                       this.drivetrain.m_robotSpeeds.withSpeeds(new ChassisSpeeds()))
                       )
                )
        );
    }
}
