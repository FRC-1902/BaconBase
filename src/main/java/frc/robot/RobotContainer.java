package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auto.AutoRoutines;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.WaypointCommands;
import frc.robot.field.AllianceFlipUtil;
import frc.robot.field.WaypointGroup;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.ControllerSubsystem.Button;
import frc.robot.subsystems.ControllerSubsystem.ControllerName;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.MapleSimUtil;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

  private static final Pose2d SIM_STARTING_POSE = new Pose2d(3.0, 4.0, Rotation2d.kZero);

  private final ControllerSubsystem controllers = ControllerSubsystem.getInstance();

  private final Drive drive;

  private final Vision vision;

  private final LoggedDashboardChooser<Command> autoChooser;

  private SwerveDriveSimulation driveSimulation;

  public RobotContainer() {
    switch (Constants.CURRENT_MODE) {
      case REAL -> {
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSpark(DriveConstants.MODULES[0]),
                new ModuleIOSpark(DriveConstants.MODULES[1]),
                new ModuleIOSpark(DriveConstants.MODULES[2]),
                new ModuleIOSpark(DriveConstants.MODULES[3]),
                pose -> {});
        vision = createVision(VisionIOPhotonVision::new);
      }
      case SIM -> {
        driveSimulation =
            new SwerveDriveSimulation(MapleSimUtil.driveTrainConfig(), SIM_STARTING_POSE);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                pose -> {
                  driveSimulation.setSimulationWorldPose(pose);
                  VisionIOPhotonVisionSim.resetSimulationPose(pose);
                });
        vision =
            createVision(
                camera ->
                    new VisionIOPhotonVisionSim(
                        camera, driveSimulation::getSimulatedDriveTrainPose));
        drive.setPose(SIM_STARTING_POSE);
      }
      default -> {
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                pose -> {});
        vision = createVision(camera -> new VisionIO() {});
      }
    }

    WaypointCommands.registerNamedCommands(drive);
    AutoRoutines.configure(drive);
    autoChooser = AutoRoutines.buildChooser(drive);
    configureBindings();
  }

  private static Vision createVision(Function<CameraConfig, VisionIO> cameraFactory) {
    VisionIO[] cameras = new VisionIO[VisionConstants.CAMERAS.length];
    for (int i = 0; i < cameras.length; i++) {
      cameras[i] =
          VisionConstants.ENABLED
              ? cameraFactory.apply(VisionConstants.CAMERAS[i])
              : new VisionIO() {};
    }
    return new Vision(cameras);
  }

  private DoubleSupplier driverForward() {
    return () -> -controllers.getCommandController(ControllerName.DRIVE).getLeftY();
  }

  private DoubleSupplier driverLeft() {
    return () -> -controllers.getCommandController(ControllerName.DRIVE).getLeftX();
  }

  private DoubleSupplier driverRotation() {
    return () -> -controllers.getCommandController(ControllerName.DRIVE).getRightX();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(drive, driverForward(), driverLeft(), driverRotation()));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.X)
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.resetHeading(
                            AllianceFlipUtil.shouldFlip() ? Rotation2d.k180deg : Rotation2d.kZero),
                    drive)
                .ignoringDisable(true)
                .alongWith(Commands.runOnce(() -> controllers.vibrate(ControllerName.DRIVE))));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.A)
        .whileTrue(WaypointCommands.holdAlignToNearest(drive, WaypointGroup.PLACEHOLDER));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.B)
        .whileTrue(
            WaypointCommands.aimAtNearest(
                drive, driverForward(), driverLeft(), WaypointGroup.PLACEHOLDER_2));

    controllers
        .getTrigger(ControllerName.DRIVE, Button.Y)
        .whileTrue(WaypointCommands.pathThenAlignToNearest(drive, WaypointGroup.PLACEHOLDER_3));

    controllers.getTrigger(ControllerName.DRIVE, Button.LB).whileTrue(drive.run(drive::stopWithX));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void setDriveBrakeMode(boolean enabled) {
    drive.setDriveBrakeMode(enabled);
  }

  public void updateSimulation() {
    if (driveSimulation == null) {
      return;
    }
    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }

  public void logDriverInputs() {
    var driver = controllers.getCommandController(ControllerName.DRIVE);
    Logger.recordOutput("Driver/RawLeftY", driver.getLeftY());
    Logger.recordOutput("Driver/RawLeftX", driver.getLeftX());
    Logger.recordOutput("Driver/RawRightX", driver.getRightX());
    Logger.recordOutput(
        "Driver/ActiveCommand",
        drive.getCurrentCommand() == null ? "none" : drive.getCurrentCommand().getName());
  }

  public Optional<Pose2d> getSimulationGroundTruthPose() {
    return Optional.ofNullable(driveSimulation)
        .map(SwerveDriveSimulation::getSimulatedDriveTrainPose);
  }

  public Drive getDrive() {
    return drive;
  }

  public Vision getVision() {
    return vision;
  }
}
