package com.team841.dory.drive.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.Drivetrain;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.*;

public class AutoSnap extends Command {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    Drivetrain drivetrain;
    ProfiledPIDController controller;
    Pose2d target;
    double angle;

    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    double count = 0;
    boolean keepgoing = true;
    double oldAngle;
    public AutoSnap(Drivetrain drivetrain) {
        this.controller = drivetrain.autoAlignController;
        this.drivetrain = drivetrain;
        driveHeading.HeadingController.setPID(34.459, 0, 2.5039);
        driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain);
        setName("AutoSnap");
    }

    @Override
    public void initialize() {
        this.target = drivetrain.getPoseToScore();
        this.controller.reset(0);
        this.angle = Math.atan2(this.target.getX() - this.drivetrain.getPose().getX(), this.target.getY() - this.drivetrain.getPose().getY());
        this.oldAngle = angle;
        runControl();
        count++;
    }

    @Override
    public void execute() {
        runControl();
        if (count==5 && keepgoing){
            this.angle = Math.atan2(this.target.getX() - this.drivetrain.getPose().getX(), this.target.getY() - this.drivetrain.getPose().getY());
            if (Math.abs(this.angle - this.oldAngle) < 0.1) {
                keepgoing = false;
            }
            this.count = 0;
        }
    }

    private void runControl() {
        Transform2d transform = this.drivetrain.getPose().minus(this.target);
        double magnitude = Math.sqrt(transform.getX() * transform.getX() + transform.getY() * transform.getY());
        double output = this.controller.calculate(0, magnitude);
        this.drivetrain.setControl(driveHeading
                .withVelocityX(output * Math.sin(angle))
                .withVelocityY(output * Math.cos(angle))
                .withTargetDirection(this.target.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    @Override
    public boolean isFinished() {
        return this.controller.atSetpoint();
    }

//    public static Transform2d transform2dFromRotation(Rotation2d rotation) {
//        return new Transform2d(Translation2d.kZero, rotation);
//    }
//
//    public static Transform2d transform2dFromTranslation(Translation2d translation) {
//        return new Transform2d(translation, Rotation2d.kZero);
//    }
//
//    public static Pose2d pose2dFromRotation(Rotation2d rotation) {
//        return new Pose2d(Translation2d.kZero, rotation);
//    }
//
//    public static Pose2d pose2dFromTranslation(Translation2d translation) {
//        return new Pose2d(translation, Rotation2d.kZero);
//    }
}
