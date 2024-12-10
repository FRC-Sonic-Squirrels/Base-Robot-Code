package frc.robot.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;
import frc.robot.Constants;

public class MechanismVisualization {
  private static final LoggerGroup logGroup = LoggerGroup.build("Visualization");
  private static final LoggerEntry.StructArray<Pose3d> logMechanism =
      logGroup.buildStructArray(Pose3d.class, "Mechanism");
  private static final LoggerEntry.Struct<Pose2d> logTestPose =
      logGroup.buildStruct(Pose2d.class, "TestPose");

  private static Pose3d mechFirstStage = Constants.zeroPose3d;
  private static Pose3d mechSecondStage = Constants.zeroPose3d;
  private static Pose3d mechTwo = Constants.zeroPose3d;
  private static Pose3d gamepiece = Constants.zeroPose3d;

  public static void updateVisualization() {
    // TODO: input positions as variables, add to pose3ds
    mechFirstStage = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
    mechSecondStage = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
    mechTwo = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
    gamepiece = new Pose3d(0.0, 0.0, 0.0, new Rotation3d());
  }

  public static void logMechanism() {
    logMechanism.info(new Pose3d[] {mechFirstStage, mechSecondStage, mechTwo, gamepiece});
    logTestPose.info(Constants.zeroPose2d);
  }
}
