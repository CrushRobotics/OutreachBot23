package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ShooterCommand extends CommandBase {
    
    final private CommandXboxController controller;
    final private ShooterSubsystem shooterSubsystem;

    public ShooterCommand(CommandXboxController controller, ShooterSubsystem shooterSubsystem) {
        this.controller = controller;
        this.shooterSubsystem = shooterSubsystem;
        this.addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        //controller.y().
        //boolean yButton = controller.y().getAsBoolean();

        //shooterSubsystem.shoot(yButton);

        shooterSubsystem.shoot(false);
    }

}
