package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
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

    /* 
    double turn = joystick.getRawAxis(XboxController.Axis.kRightX.value);
    double forw = joystick.getRawAxis(XboxController.Axis.kLeftY.value);

    // deadband gamepad 10% 
    if (Math.abs(forw) < 0.10) {
      forw = 0;
    }
    if (Math.abs(turn) < 0.10) {
      turn = 0;
    }

    double leftSpeed = (forw + turn);
    double rightSpeed = (forw - turn);

    */
    double left = controller.getLeftY();
    double right = controller.getRightY();

    left *= 0.5;
    right *= 0.5;

    driveSubsystem.tankDrive(left, right);
    //driveSubsystem.arcadeDrive(fwd, rot);

  }
}
