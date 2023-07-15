package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
    
  final private CommandXboxController controller;
  final private DriveSubsystem driveSubsystem;

  public DefaultDriveCommand(CommandXboxController controller, DriveSubsystem driveSubsystem) {
    this.controller = controller;
    this.driveSubsystem = driveSubsystem;
    this.addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {

    double fwd = controller.getLeftY();
    double rot = controller.getRightX();

    driveSubsystem.arcadeDrive(fwd, rot);

  }
}
