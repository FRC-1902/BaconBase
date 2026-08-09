package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.WaypointCommands;
import frc.robot.field.WaypointGroup;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.Button;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import java.io.File;
import org.littletonrobotics.junction.Logger;
import swervelib.SwerveInputStream;

public class RobotContainer {

  private final ControllerSubsystem controllers = ControllerSubsystem.getInstance();

  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(
          new File(Filesystem.getDeployDirectory(), Constants.Drive.SWERVE_CONFIG_DIRECTORY));

  private final Vision vision = new Vision();

  private final SendableChooser<Command> autoChooser;

  private final SwerveInputStream driveStream =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> shapeAxis(controllers.getCommandController(ControllerName.DRIVE).getLeftY()),
              () -> shapeAxis(controllers.getCommandController(ControllerName.DRIVE).getLeftX()))
          .withControllerRotationAxis(
              () ->
                  -MathUtil.applyDeadband(
                          controllers.getCommandController(ControllerName.DRIVE).getRightX(),
                          Constants.Operator.ROTATION_DEADBAND)
                      * Constants.Operator.ROTATION_SCALE)
          .scaleTranslation(Constants.Operator.TRANSLATION_SCALE)
          .allianceRelativeControl(true);

  public RobotContainer() {
    WaypointCommands.registerNamedCommands(drivebase);
    autoChooser = com.pathplanner.lib.auto.AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption(Constants.Auto.DEFAULT_AUTO_NAME, Commands.none());
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private static double shapeAxis(double input) {
    double deadbanded = MathUtil.applyDeadband(input, Constants.Operator.TRANSLATION_DEADBAND);
    return Constants.Operator.INPUT_LINEAR_WEIGHT * deadbanded
        + Constants.Operator.INPUT_CUBIC_WEIGHT * Math.pow(deadbanded, 3);
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(drivebase.driveFieldOriented(driveStream));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.X)
        .onTrue(
            drivebase
                .runOnce(drivebase::zeroGyroWithAlliance)
                .alongWith(Commands.runOnce(() -> controllers.vibrate(ControllerName.DRIVE))));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.A)
        .whileTrue(WaypointCommands.holdSnapToNearest(drivebase, WaypointGroup.PLACEHOLDER));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.B)
        .whileTrue(
            WaypointCommands.aimAtNearest(drivebase, driveStream, WaypointGroup.PLACEHOLDER_2));
    controllers
        .getTrigger(ControllerName.DRIVE, Button.Y)
        .whileTrue(WaypointCommands.pathThenSnapToNearest(drivebase, WaypointGroup.PLACEHOLDER_3));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.LB)
        .whileTrue(drivebase.run(drivebase::lock));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }

  public void logDriverInputs() {
    var driver = controllers.getCommandController(ControllerName.DRIVE);
    Logger.recordOutput("Driver/RawLeftY", driver.getLeftY());
    Logger.recordOutput("Driver/RawLeftX", driver.getLeftX());
    Logger.recordOutput("Driver/RawRightX", driver.getRightX());
    Logger.recordOutput(
        "Driver/ShapedRotation",
        -MathUtil.applyDeadband(driver.getRightX(), Constants.Operator.ROTATION_DEADBAND)
            * Constants.Operator.ROTATION_SCALE);
    Logger.recordOutput(
        "Driver/DefaultCommandActive",
        drivebase.getCurrentCommand() == drivebase.getDefaultCommand());
    Logger.recordOutput(
        "Driver/ActiveCommand",
        drivebase.getCurrentCommand() == null ? "none" : drivebase.getCurrentCommand().getName());
  }

  public void updateVision() {
    if (RobotBase.isSimulation()) {
      drivebase.getSimulationPose().ifPresent(vision::updateSimulation);
    }
    vision.updatePoseEstimation(drivebase.getSwerveDrive());
    Logger.recordOutput("Auto/Selected", String.valueOf(autoChooser.getSelected()));
  }

  public SwerveSubsystem getDrivebase() {
    return drivebase;
  }

  public Vision getVision() {
    return vision;
  }

  public SwerveInputStream getDriveStream() {
    return driveStream;
  }
}
