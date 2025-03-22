package com.team841.dory.drive.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team841.dory.constants.RC;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.Drivetrain;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import static edu.wpi.first.units.Units.*;

public class AutoSnap extends Command {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    Drivetrain drivetrain;
    ProfiledPIDController controller;
    Pose2d target = new Pose2d(0, 0, new Rotation2d(Math.PI/2));
    double atan2Angle = -(3.0/4.0) * Math.PI;
    double pointAngle;

    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

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
        this.target = this.drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar());
        this.atan2Angle = this.drivetrain.getAngleToPosePolar(target);
        this.controller.reset(0,
                Math.min(0.0, new Translation2d(
                        this.drivetrain.getChassisSpeeds().vxMetersPerSecond, this.drivetrain.getChassisSpeeds().vyMetersPerSecond)
                        .rotateBy(this.target.getTranslation().minus(
                                this.drivetrain.getPose().getTranslation()).getAngle())
                        .getX())
                );

        if (Math.abs(this.atan2Angle) > Math.PI/2){
            this.pointAngle = (Math.PI/2 - (Math.PI - Math.abs(atan2Angle)));
        } else {
            this.pointAngle = Math.PI/2 - atan2Angle;
        }

        Logger.recordOutput("AutoSnap/pointAngle", this.pointAngle);
        runControl();
    }

    @Override
    public void execute() {
        runControl();
    }

    private void runControl() {
        Transform2d transform = this.drivetrain.getPose().minus(this.target);
        double magnitude = this.drivetrain.getPose().getTranslation().getDistance(this.target.getTranslation());
        this.controller.reset(
                0.0,
                this.controller.getSetpoint().velocity
        );
        double output = this.controller.calculate(0, magnitude);
        Logger.recordOutput("AutoSnap/output", output);

        double x, y;
        x = magnitude * Math.sin(pointAngle);
        y = magnitude * Math.cos(pointAngle);

        Logger.recordOutput("AutoSnap/x", x);
        Logger.recordOutput("AutoSnap/y", y);

        Logger.recordOutput("AutoSnap/ControllerSetpointPosition", this.controller.getSetpoint().position);
        Logger.recordOutput("AutoSnap/ControllerSetpointVelocity", this.controller.getSetpoint().velocity);

        if (!RC.isRedAlliance.get()){
            this.drivetrain.setControl(
                    this.driveHeading
                            .withVelocityY(this.atan2Angle < 0 ? y : -y)
                            .withVelocityX(Math.abs(this.atan2Angle) > Math.PI/2 ? x : -x)
                            .withTargetDirection(this.target.getRotation())
            );
        } else {
            this.drivetrain.setControl(
                    this.driveHeading
                            .withVelocityY(this.atan2Angle < 0 ? -y : y)
                            .withVelocityX(Math.abs(this.atan2Angle) > Math.PI/2 ? -x : x)
                            .withTargetDirection(this.target.getRotation())
            );
        }
    }

    @Override
    public void end(boolean interrupted) {
        return;
    }

    @Override
    public boolean isFinished() {
        return this.controller.atSetpoint();
    }

    public double getChassisSpeedsToTarget(){
        ChassisSpeeds fieldRelativeSpeeds =
                ChassisSpeeds.fromRobotRelativeSpeeds(
                        this.drivetrain.getChassisSpeeds(),
                        this.drivetrain.getPose().getRotation());


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
