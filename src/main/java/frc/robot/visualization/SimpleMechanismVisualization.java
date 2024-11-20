package frc.robot.visualization;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.team2930.LoggerEntry;
import frc.lib.team2930.LoggerGroup;

public class SimpleMechanismVisualization {
  private static final LoggerGroup logGroup = LoggerGroup.build("Mechanism");
  private static final LoggerEntry.Mechanism logMech = logGroup.buildMechanism2d("SimpleMechanism");

  static Mechanism2d mechanism2d =
      new Mechanism2d(
          Units.Inches.of(32.0).in(Units.Meters), Units.Inches.of(50.0).in(Units.Meters));

  static MechanismRoot2d mechRoot =
      mechanism2d.getRoot(
          "mechRoot", Units.Inches.of(4).in(Units.Meters), Units.Inches.of(4).in(Units.Meters));

  static MechanismLigament2d mechLigament =
      mechRoot.append(
          new MechanismLigament2d(
              "mech",
              0, // TODO: change to length of ligament
              0 // TODO: change to initial angle of ligament
              ));

  static {
    mechLigament.setColor(new Color8Bit(0, 0, 255));
    mechLigament.setLineWeight(15.0);
  }

  public static void updateVisualization(Rotation2d mechAngle) {
    mechLigament.setAngle(new Rotation2d(-mechAngle.getRadians()));
  }

  public static void logMechanism() {
    logMech.info(mechanism2d);
  }
}
