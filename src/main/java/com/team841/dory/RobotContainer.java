package com.team841.dory;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team254.vision.VisionFieldPoseEstimate;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.*;
import com.team841.dory.drive.Commands.DriveMaintainHeading;

import com.team841.dory.escalator.Escalator;
import com.team841.dory.escalator.EscalatorIO;
import com.team841.dory.escalator.EscalatorIOKraken;
import com.team841.dory.flapSystem.FlapSystem;
import com.team841.dory.flapSystem.FlapSystemIO;
import com.team841.dory.flapSystem.FlapSystemIOKraken;
import com.team841.dory.shooter.Shooter;
import com.team841.dory.shooter.ShooterIO;
import com.team841.dory.shooter.ShooterIOKraken;
import com.team841.dory.vision.Vision;
import com.team841.dory.vision.VisionIO;
import com.team841.dory.vision.VisionIOLimelights;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);
    private final CommandXboxController cojoystick = new CommandXboxController(1);

    public final Drivetrain drivetrain;

    public final VisionIO visionIO;
    public final Vision vision;

    public final EscalatorIO escalatorIO;
    public final Escalator escalator;

    public final ShooterIO shooterIO;
    public final Shooter shooter;

    public final FlapSystemIO flapSystemIO;
    public final FlapSystem flapSystem;

    public final Control control;
//
    private final DriveMaintainHeading driveMaintainHeading;
    private final Command escalatorDefaultCommand;

    private final SendableChooser<Command> autoChooser;

    public final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer = new Consumer<>() {
        @Override
        public void accept(VisionFieldPoseEstimate visionFieldPoseEstimate) {
//           drivetrain.addVisionMeasurement(visionFieldPoseEstimate);
             return;
        }
    };


    public RobotContainer() {
        switch (RC.robotType) {
            case SIM -> {
                this.drivetrain = new Drivetrain(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                this.visionIO = new VisionIOLimelights();
                this.vision = new Vision(visionIO, visionEstimateConsumer, drivetrain);

                this.escalatorIO = new EscalatorIOKraken();
                this.escalator = new Escalator(escalatorIO);

                this.shooterIO = new ShooterIOKraken();
                this.shooter = new Shooter(shooterIO);

                this.flapSystemIO = new FlapSystemIOKraken();
                this.flapSystem = new FlapSystem(flapSystemIO);

                this.control = new Control(this.drivetrain, this.escalator, this.shooter, this.flapSystem);
            }
            default -> {
                this.drivetrain = new Drivetrain(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                this.visionIO = new VisionIOLimelights();
                this.vision = new Vision(visionIO, visionEstimateConsumer, drivetrain);

                this.escalatorIO = new EscalatorIOKraken();
                this.escalator = new Escalator(escalatorIO);

                this.shooterIO = new ShooterIOKraken();
                this.shooter = new Shooter(shooterIO);

                this.flapSystemIO = new FlapSystemIOKraken();
                this.flapSystem = new FlapSystem(flapSystemIO);

                this.control = new Control(this.drivetrain, this.escalator, this.shooter, this.flapSystem);
            }
        }

        this.driveMaintainHeading = new DriveMaintainHeading(
                drivetrain, () -> -joystick.getLeftY(), () -> -joystick.getLeftX(), () -> -joystick.getRightX(), ()->joystick.L2().getAsBoolean());

        this.escalatorDefaultCommand = this.escalator.passiveHoldDown();

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(driveMaintainHeading);
        escalator.setDefaultCommand(escalatorDefaultCommand);

        joystick.touchpad().onTrue(new InstantCommand(drivetrain::seedFieldCentric));

        /* Zero Automation */
        cojoystick.y()
                .and(cojoystick.rightTrigger())
                .whileTrue(control.noSnapAutoScoreL4);

        cojoystick.y()
                .and(cojoystick.rightBumper())
                .whileTrue(control.noSnapAutoScoreL3);

        cojoystick.y()
                .and(cojoystick.leftTrigger())
                .whileTrue(control.noSnapAutoScoreL2);

        cojoystick.y()
                .onFalse(control.escalatorGoHome);

        /* Automated */
        cojoystick.leftBumper()
                .and(cojoystick.rightTrigger())
                .whileTrue(control.snapScoreL4);

        cojoystick.leftBumper()
                .and(cojoystick.rightBumper())
                .whileTrue(control.snapScoreL3);

        cojoystick.leftBumper()
                .and(cojoystick.leftTrigger())
                .whileTrue(control.snapScoreL2);

        cojoystick.leftBumper()
                .onFalse(control.escalatorGoHome);

        /* ###################################### */

        cojoystick.x()
                .and(() -> !this.shooter.shooterHasCoral())
                .whileTrue(control.intake);

        cojoystick.b()
                .whileTrue(new InstantCommand(()->this.shooter.setDutyCycle(-.08),shooter))
                .onFalse(new InstantCommand(()->this.shooter.setDutyCycle(0),shooter));

        cojoystick.back()
                .onTrue(new InstantCommand(escalator::zero, escalator));

        cojoystick.povDown()
                .whileTrue(this.escalator.goDown());

        cojoystick.povUp()
                .whileTrue(this.escalator.goUp());
        
        cojoystick.povLeft()
                .whileTrue(new InstantCommand(()->this.flapSystem.setFlapperDutyCycle(0.25),flapSystem))
                .onFalse(new InstantCommand(()->this.flapSystem.stopFlapper(),flapSystem));
             
        cojoystick.povRight()
                .whileTrue(new InstantCommand(()->this.flapSystem.setFlapperDutyCycle(-0.25),flapSystem))
                .onFalse(new InstantCommand(()->this.flapSystem.stopFlapper(),flapSystem));
        joystick.povUp()
                .whileTrue(new InstantCommand(()->this.flapSystem.setHangDutyCycle(-.7),flapSystem))
                .onFalse(new InstantCommand(()->this.flapSystem.stopHang(),flapSystem));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
