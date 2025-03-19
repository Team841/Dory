package com.team841.dory;

import static edu.wpi.first.units.Units.*;

import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.team254.vision.VisionFieldPoseEstimate;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.DriveIO;
import com.team841.dory.drive.DriveIOReal;
import com.team841.dory.drive.Drivetrain;
import com.team841.dory.drive.Commands.DriveMaintainHeading;

import com.team841.dory.escalator.Escalator;
import com.team841.dory.escalator.EscalatorIO;
import com.team841.dory.escalator.EscalatorIOKraken;
import com.team841.dory.shooter.Shooter;
import com.team841.dory.shooter.ShooterIO;
import com.team841.dory.shooter.ShooterIOKraken;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    private final CommandPS5Controller joystick = new CommandPS5Controller(0);

    public final DriveIO driveIO;
    public final Drivetrain drivetrain;

    // public final VisionIO visionIO;
    // public final Vision vision;


    public final EscalatorIO escalatorIO;
    public final Escalator escalator;

    public final ShooterIO shooterIO;
    public final Shooter shooter;

    public final Control control;

    private final DriveMaintainHeading driveMaintainHeading;

    private final SendableChooser<Command> autoChooser;

    public final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer = new Consumer<VisionFieldPoseEstimate>() {
        @Override
        public void accept(VisionFieldPoseEstimate visionFieldPoseEstimate) {
            drivetrain.addVisionMeasurement(visionFieldPoseEstimate);
        }
    };

    public RobotContainer() {
        switch (RC.robotType) {
            default -> {
                this.driveIO = new DriveIOReal(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
                this.drivetrain = new Drivetrain(driveIO);

                // this.visionIO = new VisionIOLimelights();
                // this.vision = new Vision(visionIO, visionEstimateConsumer, drivetrain);

                this.escalatorIO = new EscalatorIOKraken();
                this.escalator = new Escalator(escalatorIO);

                this.shooterIO = new ShooterIOKraken();
                this.shooter = new Shooter(shooterIO);

                this.control = new Control(this.drivetrain, this.escalator, this.shooter);
            }
        }

        this.driveMaintainHeading = new DriveMaintainHeading(
                drivetrain, () -> -joystick.getLeftY(), () -> -joystick.getLeftX(), () -> -joystick.getRightX());

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(driveMaintainHeading);

        joystick.create().onTrue(new InstantCommand(drivetrain::seedFieldCentric));

        /* Zero Automation */
        joystick.options()
                .and(joystick.R2())
                .whileTrue(control.noSnapAutoScoreL4);

        joystick.options()
                .and(joystick.R1())
                .whileTrue(control.noSnapAutoScoreL3);

        joystick.options()
                .and(joystick.L2())
                .whileTrue(control.noSnapAutoScoreL2);

        joystick.options()
                .onFalse(control.escalatorGoHome);

        /* Automated */
        joystick.L1()
                .and(joystick.R2())
                .whileTrue(control.snapScoreL4);

        joystick.L1()
                .and(joystick.R1())
                .whileTrue(control.snapScoreL3);

        joystick.L1()
                .and(joystick.L2())
                .whileTrue(control.snapScoreL2);

        joystick.L1()
                .onFalse(control.escalatorGoHome);

        /* ###################################### */

        joystick.L1()
                .and(() -> !this.shooter.shooterHasCoral())
                .whileTrue(control.intake);

        joystick.create()
                .onTrue(new InstantCommand(escalator::zero, escalator));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
