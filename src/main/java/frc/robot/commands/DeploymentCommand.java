package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DeploymentSubsystem;

public class DeploymentCommand extends CommandBase {

    private long deploymentTime = 1000;
    private long startDeploymentTime = 0;
    private DeploymentSubsystem deploymentSubsystem;
    
    public DeploymentCommand(DeploymentSubsystem deploymentSubsystem) {
        this.deploymentSubsystem = deploymentSubsystem;
        this.addRequirements(deploymentSubsystem);
    }

    @Override
    public void initialize() {
        startDeploymentTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        deploymentSubsystem.deploy();
    }

    @Override
    public boolean isFinished() {
        return startDeploymentTime + deploymentTime > System.currentTimeMillis();
    }

    @Override
    public void end(boolean interrupted) {
        deploymentSubsystem.stop();
    }
}

