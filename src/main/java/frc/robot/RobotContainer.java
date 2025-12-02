// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.constants.TunerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import static frc.robot.constants.SubsystemConstants.DrivetrainConstants.TeleConstants.MAX_TRANSLATION;
import static frc.robot.constants.FieldConstants.*;
import frc.robot.Telemetry;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();
  }
  private final Telemetry logger = new Telemetry(MAX_TRANSLATION);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private void configureBindings() {
    if(!Robot.isReal()) {
      drivetrain.registerTelemetry(logger::telemeterize);
    }
  }

  public Command getAutonomousCommand() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAutonomousCommand'");
  }

}
