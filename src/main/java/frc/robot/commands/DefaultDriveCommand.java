package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
public class DefaultDriveCommand extends CommandBase {
  
  final private CommandXboxController controller;
  final private DriveSubsystem driveSubsystem;
  private int speedRange = 3;
  private long rotateTime = 1000;
  private long startRotateTime = 0 ;
  private boolean upPressed = false;
  private boolean downPressed = false;
  private boolean leftPressed = false;
  private boolean rightPressed = false;
  private boolean isRotatingLeft = false;
  private boolean isRotatingRight = false;

  public DefaultDriveCommand(CommandXboxController controller, DriveSubsystem driveSubsystem) {
    this.controller = controller;
    this.driveSubsystem = driveSubsystem;
    this.addRequirements(driveSubsystem);
  }

  @Override
  public void execute() {

    boolean isTankDrive = SmartDashboard.getBoolean("Tank Drive", false);
    if(isTankDrive) {
      //Beginning of Chain Drive code
      double left = controller.getLeftY();
      double right = controller.getRightY();
  
      left = left * 0.4;
      right = right * 0.4;
  
      // Decrease the speed if bumper is pressed
      if (controller.rightBumper().getAsBoolean()){
        left = left * 0.5;
        right = right * 0.5;
      }
    
      //Decrease the speed in incrments when the D-pad is pressed
      if(controller.povUp().getAsBoolean() == true && !upPressed){
        upPressed = true;
        if(speedRange < 5){
          speedRange = speedRange + 1;
        }
      }
      else if (controller.povUp().getAsBoolean() == false && upPressed){
        upPressed = false;
      }
      if(controller.povDown().getAsBoolean() == true && !downPressed){
        downPressed = true;
        if(speedRange > 0){
          speedRange = speedRange - 1;
        }
      } else if (controller.povDown().getAsBoolean() == false && downPressed) {
        downPressed = false;
      }

      left = left * (speedRange/5.0);
      right = right * (speedRange/5.0);
      driveSubsystem.tankDrive(left, right);
    }
     else {
      //Beginning of Arcade Drive code
      double xAxis = controller.getRightX();
      double yAxis = controller.getLeftY();
      xAxis = xAxis * 0.6;
      yAxis = yAxis * 0.6;
  
       // Decrease the speed if bumper is pressed
       if (controller.rightBumper().getAsBoolean()){
         xAxis = xAxis * 0.5;
         yAxis = yAxis * 0.5;
       }
       // Decrease the speed on fine incrments if up and down on D-Pad are pressed(Arcade Drive)
      if(controller.povUp().getAsBoolean() == true && !upPressed){
        upPressed = true;
        if(speedRange < 5){
          speedRange = speedRange + 1;
        }
      }
      else if (controller.povUp().getAsBoolean() == false && upPressed){
        upPressed = false;
      }
      if(controller.povDown().getAsBoolean() == true && !downPressed){
        downPressed = true;
        if(speedRange > 0){
          speedRange = speedRange - 1;
        }
      } else if (controller.povDown().getAsBoolean() == false && downPressed) {
        downPressed = false;
      }
  
      xAxis = xAxis * (speedRange / 5.0);
      yAxis = yAxis * (speedRange / 5.0);
  
      // Turns the robot left 90 degrees(Arcade Drive)
      if(controller.povLeft().getAsBoolean() == true && !leftPressed){
        leftPressed = true;
        isRotatingLeft = true;
        startRotateTime = System.currentTimeMillis();
      } else if (controller.povLeft().getAsBoolean() == false && leftPressed) {
        leftPressed = false;
      }
      if(isRotatingLeft == true){
        if(startRotateTime + rotateTime > System.currentTimeMillis()){
          yAxis = 0;
          xAxis = -1 * (speedRange / 5.0) ;
        }
        else {
          isRotatingLeft = false;
          xAxis = 0;
        }
      }
      //Rotates right 90 degrees(Arcade Drive)
      if(controller.povRight().getAsBoolean() == true && !rightPressed){
        rightPressed = true;
        isRotatingRight = true;
        startRotateTime = System.currentTimeMillis();
      } else if (controller.povRight().getAsBoolean() == false && rightPressed) {
        rightPressed = false;
      }
      if(isRotatingRight == true){
        if(startRotateTime + rotateTime > System.currentTimeMillis()){
          yAxis = 0;
          xAxis = 1 * (speedRange / 5.0);
        }
        else{
          isRotatingRight = false;
          xAxis = 0;
        }
      }
      
      driveSubsystem.arcadeDrive(xAxis, yAxis);
    }  
  }
}
