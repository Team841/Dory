package com.team841.dory.drive.Commands;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team841.dory.constants.TunerConstants;
import com.team841.dory.drive.Drivetrain;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class AutoSnap extends Command {

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    Drivetrain drivetrain;
    ProfiledPIDController vx;
    ProfiledPIDController vy;
    Pose2d target = new Pose2d(1, 1, new Rotation2d(Math.PI/2));

    private final SwerveRequest.FieldCentricFacingAngle driveHeading = new SwerveRequest.FieldCentricFacingAngle()// Add a 10% deadband
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    public AutoSnap(Drivetrain drivetrain) {
        this.vx = drivetrain.vxController;
        this.vy = drivetrain.vyController;
        this.drivetrain = drivetrain;

        addRequirements(this.drivetrain);
        setName("AutoSnap");
    }

    @Override
    public void initialize() {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
//        this.target = drivetrain.getPoseToScore(this.drivetrain.getAngleToReefPolar());
        Logger.recordOutput("AutoSnap/PoseTarget", this.target);
        Translation2d translation2d = this.drivetrain.getPose().getTranslation();
//        Translation2d error = this.target.minus(this.drivetrain.getPose()).getTranslation();
        this.vx.reset(translation2d.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
        this.vy.reset(translation2d.getY(), fieldRelativeSpeeds.vyMetersPerSecond);
        runAutoSnap();
    }

    @Override
    public void execute() {
        runAutoSnap();
    }

    private void runAutoSnap() {
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                this.drivetrain.getChassisSpeeds(), this.drivetrain.getPose().getRotation());
        Translation2d translation2d = this.drivetrain.getPose().getTranslation();
//        Translation2d error = this.target.minus(this.drivetrain.getPose()).getTranslation();
//        this.vx.reset(translation2d.getX(), fieldRelativeSpeeds.vxMetersPerSecond);
//        this.vy.reset(translation2d.getY(), fieldRelativeSpeeds.vyMetersPerSecond);

//        double outputX = this.vx.calculate(0, error.getX());
//        double outputY = this.vy.calculate(0, error.getY());

        double outputX = this.vx.calculate(translation2d.getX(), this.target.getX()) + this.vx.getSetpoint().velocity;
        double outputY = this.vy.calculate(translation2d.getY(), this.target.getY()) + this.vy.getSetpoint().velocity;

        outputX *= 0.7;
        outputY *= 0.7;

        Logger.recordOutput("AutoSnap/outputX", outputX);
        Logger.recordOutput("AutoSnap/outputY", outputY);
        Logger.recordOutput("AutoSnap/vxAtGoal", this.vx.atGoal());
        Logger.recordOutput("AutoSnap/vyAtGoal", this.vy.atGoal());

        // outputX *= RC.isRedAlliance.get() ? 1 : -1;
        // outputY *= RC.isRedAlliance.get() ? 1 : -1;

//        if (!RC.isRedAlliance.get()){
        this.drivetrain.setControl(
                this.driveHeading.withVelocityY(outputY).withVelocityX(outputX).withTargetDirection(this.target.getRotation())
        );
//        } else {
//            this.drivetrain.setControl(
//                    this.driveHeading
//                            .withVelocityY(-outputY)
//                            .withVelocityX(outputX)
//                            .withTargetDirection(this.target.getRotation())
//            );
//        }
    }


    @Override
    public void end(boolean interrupted) {
        this.drivetrain.setControl(this.drivetrain.m_robotSpeeds.withSpeeds(new ChassisSpeeds()));
    }

    @Override
    public boolean isFinished() {
        return this.vx.atGoal() && this.vy.atGoal();
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
