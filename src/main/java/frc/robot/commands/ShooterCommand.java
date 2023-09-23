package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class ShooterCommand extends CommandBase {
    
    final private XboxController controller;
    final private ShooterSubsystem shooterSubsystem;

    public DefaultShooterCommand(XboxController controller, ShooterSubsystem shooterSubsystem) {
        this.controller = controller;
        this.shooterSubsystem = shooterSubsystem;
        this.addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {

        boolean yButton = controller.getYButton();

        shooterSubsystem.shoot(yButton);
    }

}
