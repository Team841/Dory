package com.team841.dory;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.Commands.AutoSnap;
import com.team841.dory.drive.Drivetrain;
import com.team841.dory.escalator.Escalator;
import com.team841.dory.escalator.MoveCommand;
import com.team841.dory.escalator.Escalator.Position;
import com.team841.dory.flapSystem.FlapSystem;
import com.team841.dory.shooter.Shooter;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static edu.wpi.first.units.Units.*;

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
                new AutoSnap(this.drivetrain), new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false)).withTimeout(1)).onlyIf(this.shooter::escalatorClear)
        );
        NamedCommands.registerCommand("GoDown", new InstantCommand(() -> this.escalator.setPosition(Position.HomeAndIntake, false), escalator));
        NamedCommands.registerCommand("Intake", Intake());
        NamedCommands.registerCommand("PassiveRaise", new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.Hold, true)));
        NamedCommands.registerCommand("FlapDown", new RunCommand(() -> this.flapSystem.setFlapperDutyCycle(-0.5), flapSystem).withTimeout(1).finallyDo(flapSystem::stopFlapper));

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
                AutoSnapInline(), new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);

        this.snapScoreL3 = new SequentialCommandGroup(
                AutoSnapInline(), new MoveCommand(this.escalator, Escalator.Position.L3, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);

        this.snapScoreL2 = new SequentialCommandGroup(
                AutoSnapInline(), new MoveCommand(this.escalator, Escalator.Position.L2, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L2, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);

        this.noSnapAutoScoreL4 = new SequentialCommandGroup(
                new MoveCommand(this.escalator, Escalator.Position.L4, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L4, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);
        this.noSnapAutoScoreL3 = new SequentialCommandGroup(
                new MoveCommand(this.escalator, Escalator.Position.L3, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);
        this.noSnapAutoScoreL2 = new SequentialCommandGroup(
                new MoveCommand(this.escalator, Escalator.Position.L2, this.shooter::shooterHasCoral, this.shooter::escalatorClear), this.shooter.runShooterScore(Escalator.Position.L3, scoreTimeout), new InstantCommand(() -> this.escalator.setPosition(Escalator.Position.HomeAndIntake, false))).onlyIf(this.shooter::escalatorClear);

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
                        this.escalator.passiveHoldDown().withTimeout(0.2), new ParallelRaceGroup(
                                this.shooter.runShooterIntake(), this.flapSystem.runIntake()
                        )

                ).onlyIf(this.shooter::escalatorClear), () -> this.escalator.atPosition(Escalator.Position.HomeAndIntake)
        );
    }

    Optional<Trajectory<SwerveSample>> z_c = Choreo.loadTrajectory("Z_c");
    Optional<Trajectory<SwerveSample>> Z_3c = Choreo.loadTrajectory("Z_3c");
    Optional<Trajectory<SwerveSample>> Z_3k = Choreo.loadTrajectory("Z_3k");
    Optional<Trajectory<SwerveSample>> Z_k3 = Choreo.loadTrajectory("Z_k3");
    Optional<Trajectory<SwerveSample>> Z_3I = Choreo.loadTrajectory("Z_3l");
    Optional<Trajectory<SwerveSample>> Z_I3 = Choreo.loadTrajectory("Z_l3");
    Optional<Trajectory<SwerveSample>> Z_3a = Choreo.loadTrajectory("Z_3a");

    public Command followPath(Trajectory traj){
        Timer timer = new Timer();
        timer.restart();
        Pose2d startingPose = traj.getPoses()[0];
        return Commands.runOnce(
                () -> drivetrain.setPose(startingPose)
        ).andThen(
                Commands.run(
                ()->{
                    Optional<SwerveSample> sample = traj.sampleAt(timer.get(), RC.isRedAlliance.get());
                    sample.ifPresent(this.drivetrain::followTrajectory);
                }
        ).until(
                () -> traj.getFinalPose(RC.isRedAlliance.get()).get() == drivetrain.getPose()
        ).withTimeout(
                traj.getTotalTime()
        ));
    }
    public Command LeftSideCrazyAuto() {
        return Commands.sequence(
                Commands.race(followPath(z_c.get())),
                NamedCommands.getCommand("AutoL4"),
                followPath(Z_3c.get()),
                NamedCommands.getCommand("Intake"),
                followPath(Z_3k.get()),
                NamedCommands.getCommand("AutoL4"),
                followPath(Z_k3.get()),
                NamedCommands.getCommand("Intake"),
                followPath(Z_3I.get()),
                NamedCommands.getCommand("AutoL4"),
                followPath(Z_I3.get()),
                NamedCommands.getCommand("Intake"),
                followPath(Z_3a.get()),
                NamedCommands.getCommand("AutoL4")
        );
    }

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public Command AutoSnapInline(){
        ProfiledPIDController vx = drivetrain.vxController;
        ProfiledPIDController vy = drivetrain.vyController;
        Pose2d target;

        return Commands.runOnce(
                () -> {
                    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                            this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
                    target = drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar());
                    Logger.recordOutput("AutoSnap/PoseTarget", target);
                    Translation2d translation2d = this.drivetrain.getPose().getTranslation();
//        Translation2d error = this.target.minus(this.drivetrain.getPose()).getTranslation();
                    vx.reset(translation2d.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
                    vy.reset(translation2d.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
                }
        ).andThen(
                Commands.run(
                        () -> this.drivetrain.setControl(drivetrain.driveHeading.withTargetDirection(target.getRotation()))
                ).until(
                        this.drivetrain.driveHeading.HeadingController::atSetpoint
                ).andThen(
                       Commands.run(
                               () -> {
                                   ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                                           this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
                                   Translation2d translation2d = this.drivetrain.getPose().getTranslation();

                                   double outputX = vx.calculate(translation2d.getX(), target.getX()) + vx.getSetpoint().velocity;
                                   double outputY = vy.calculate(translation2d.getY(), target.getY()) + vy.getSetpoint().velocity;

                                   outputX *= 0.7;
                                   outputY *= 0.7;

                                   Logger.recordOutput("AutoSnap/outputX", outputX);
                                   Logger.recordOutput("AutoSnap/outputY", outputY);
                                   Logger.recordOutput("AutoSnap/vxAtGoal", vx.atGoal());
                                   Logger.recordOutput("AutoSnap/vyAtGoal", vy.atGoal());

                                   this.drivetrain.setControl(this.drivetrain.driveHeading.withVelocityY(outputY).withVelocityX(outputX).withTargetDirection(target.getRotation())
                                   );
                               }
                       ).until(
                               () -> {return vx.atGoal() && vy.atGoal();}
                       ).finallyDo(
                               () -> this.drivetrain.setControl(this.drivetrain.m_robotSpeeds.withSpeeds(new ChassisSpeeds()))
                       )
                )
        );
    }
}
