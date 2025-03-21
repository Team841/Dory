package com.team841.dory.drive.Commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team841.dory.drive.Drivetrain;
import com.team841.dory.vision.Vision;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class DriveToPose extends Command {
    private final ProfiledPIDController driveController = new ProfiledPIDController(
            0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), 0.02);
    private Drivetrain driveSubsystem;
    private Supplier<Pose2d> poseSupplier;
    private Translation2d lastSetpointTranslation;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.2, ffMaxRadius = 0.8;

    public DriveToPose(Drivetrain driveSubsystem, Supplier<Pose2d> poseSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.poseSupplier = poseSupplier;
        addRequirements(driveSubsystem);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = poseSupplier.get();
        driveController.reset(
                currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation()),
                Math.min(
                        0.0,
                        -new Translation2d(driveSubsystem.getChassisSpeeds().vxMetersPerSecond,
                                driveSubsystem.getChassisSpeeds().vyMetersPerSecond)
                                .rotateBy(
                                        poseSupplier
                                                .get()
                                                .getTranslation()
                                                .minus(driveSubsystem.getPose().getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        thetaController.reset(currentPose.getRotation().getRadians(),
                driveSubsystem.getChassisSpeeds().omegaRadiansPerSecond);
        lastSetpointTranslation = driveSubsystem.getPose().getTranslation();
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();
        Pose2d targetPose = poseSupplier.get();

        Logger.recordOutput("DriveToPose/currentPose", currentPose);
        Logger.recordOutput("DriveToPose/targetPose", targetPose);

        double currentDistance = currentPose.getTranslation().getDistance(poseSupplier.get().getTranslation());
        double ffScaler = MathUtil.clamp(
                (currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius),
                0.0,
                1.0);
        driveErrorAbs = currentDistance;
        driveController.reset(
                lastSetpointTranslation.getDistance(targetPose.getTranslation()),
                driveController.getSetpoint().velocity);
        double driveVelocityScalar = driveController.getSetpoint().velocity * ffScaler
                + driveController.calculate(driveErrorAbs, 0.0);
        if (currentDistance < driveController.getPositionTolerance())
            driveVelocityScalar = 0.0;
        lastSetpointTranslation = new Pose2d(
                targetPose.getTranslation(),
                currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(
                        transform2dFromTranslation(
                                new Translation2d(driveController.getSetpoint().position, 0.0)))
                .getTranslation();

        // Calculate theta speed
        double thetaVelocity = thetaController.getSetpoint().velocity * ffScaler
                + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        thetaErrorAbs = Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getRadians());
        if (thetaErrorAbs < thetaController.getPositionTolerance())
            thetaVelocity = 0.0;

        // Command speeds
        var driveVelocity =
                pose2dFromRotation(currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle())
                .transformBy(transform2dFromTranslation(new Translation2d(driveVelocityScalar, 0.0)))
                .getTranslation();
        driveSubsystem.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation())));
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setControl(new SwerveRequest.ApplyRobotSpeeds());
    }

    @Override
    public boolean isFinished() {
        return poseSupplier.get().equals(null) || (driveController.atGoal() && thetaController.atGoal());
    }

    public static Transform2d transform2dFromRotation(Rotation2d rotation) {
        return new Transform2d(Translation2d.kZero, rotation);
    }

    public static Transform2d transform2dFromTranslation(Translation2d translation) {
        return new Transform2d(translation, Rotation2d.kZero);
    }

    public static Pose2d pose2dFromRotation(Rotation2d rotation) {
        return new Pose2d(Translation2d.kZero, rotation);
    }

    public static Pose2d pose2dFromTranslation(Translation2d translation) {
        return new Pose2d(translation, Rotation2d.kZero);
    }
}
