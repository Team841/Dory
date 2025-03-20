package com.team841.dory.escalator;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.BooleanSupplier;

public class MoveCommand extends Command {
    public MoveCommand(Escalator escalator, Escalator.Position position, BooleanSupplier hasCoralSupplier, BooleanSupplier isClear) {
        this.escalator = escalator;
        this.position = position;
        this.hasCoralSupplier = hasCoralSupplier;
        this.isClearSupplier = isClear;

        addRequirements(this.escalator);
        setName("Move");
    }

    Escalator escalator;
    Escalator.Position position;
    BooleanSupplier hasCoralSupplier, isClearSupplier;
    boolean hasCoral, isClear;

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.hasCoral = hasCoralSupplier.getAsBoolean();
        this.isClear = isClearSupplier.getAsBoolean();
        if (isClear)
            escalator.setPosition(position, hasCoral);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (isClear)
            this.escalator.setPosition(position, hasCoral);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.escalator.atPosition(position) || !this.isClear;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }
}
