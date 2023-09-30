package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooterCommand extends CommandBase {
    private ShooterSubsystem subsystem;

    public StopShooterCommand (ShooterSubsystem subsystem)
    {
        this.addRequirements(subsystem);
        this.subsystem = subsystem;
    }

    @Override
    public void execute()
    {
        subsystem.shoot(false);
    }

    @Override
    public boolean isFinished()
    {
        return true;
    }
}
