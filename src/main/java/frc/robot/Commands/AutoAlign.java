// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlign extends Command {
  private DriveSubsystem driveSubsystem;

  //FIXME need to be tuned to your robot. 
  private ProfiledPIDController xController = new ProfiledPIDController(0, 0, 0, new Constraints(0, 0));
  /** Creates a new AutoAlign. */
  public AutoAlign(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    xController.setTolerance(0); //FIXME how close you want to be to the tag before stopping.
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.reset(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetX = LimelightHelpers.getTargetPose3d_CameraSpace("LIMELIGHT_NAME_GOES_HERE").getMeasureX().in(Meters);

    //FIXME the goal value the second parameter is how far to the right you want the robot to go from the tag. A negitive value will be to the left.
    double xSpeed = xController.calculate(targetX, 0); 

    driveSubsystem.drive(xSpeed, xSpeed, xSpeed, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
