package frc.robot.commands;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
=======
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends CommandBase {
    
<<<<<<< HEAD
  final private XboxController controller;
  final private DriveSubsystem driveSubsystem;

  public DefaultDriveCommand(XboxController controller, DriveSubsystem driveSubsystem) {
=======
  final private CommandXboxController controller;
  final private DriveSubsystem driveSubsystem;

  public DefaultDriveCommand(CommandXboxController controller, DriveSubsystem driveSubsystem) {
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef
    this.controller = controller;
    this.driveSubsystem = driveSubsystem;
    this.addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {

<<<<<<< HEAD
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
=======
>>>>>>> 01ea2f83f98265c1c412afd23ff53a949267c9ef
    double fwd = controller.getLeftY();
    double rot = controller.getRightX();

    driveSubsystem.arcadeDrive(fwd, rot);

  }
}
