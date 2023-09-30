package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooterCommand extends CommandBase {
    private ShooterSubsystem subsystem;

    public StartShooterCommand (ShooterSubsystem subsystem)
    {
        this.addRequirements(subsystem);
        this.subsystem = subsystem;
    }

    @Override
    public void execute()
    {
        subsystem.shoot(true);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
