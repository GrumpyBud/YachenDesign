package first.robot.auto;

import static first.robot.auto.ChargedUpField.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import bline.path.Path;
import bline.path.Path.EventTrigger;
import bline.path.Path.RotationTarget;
import bline.path.Path.TranslationTarget;
import bline.path.Path.Waypoint;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Translation2d;

/**
 * Geometry checks on every leg a scoring plan can produce, for all nodes and staged pieces. These
 * check the authored lines, not how closely BLine follows them; the simulation tests cover that.
 */
class CyclePathsTest {
  @Test
  void everyLegKeepsTheRobotOffTheChargeStationAndInsideTheField() {
    for (var leg : CyclePaths.allLegs().entrySet()) {
      var points = translations(leg.getValue());
      var turning = turningSegments(leg.getValue());
      for (int i = 0; i + 1 < points.size(); i++) {
        // A turning robot sweeps its corners; otherwise its sides are a half-length away.
        double reach = turning.get(i) ? HALF_DIAGONAL : HALF_LENGTH + 0.1;
        var a = points.get(i);
        var b = points.get(i + 1);
        for (double t = 0; t <= 1.0; t += 0.01) {
          var p = a.interpolate(b, t);
          double dx =
              Math.max(
                  0, Math.max(CHARGE_STATION_MIN_X - p.getX(), p.getX() - CHARGE_STATION_MAX_X));
          double dy =
              Math.max(
                  0, Math.max(CHARGE_STATION_MIN_Y - p.getY(), p.getY() - CHARGE_STATION_MAX_Y));
          assertTrue(
              Math.hypot(dx, dy) >= reach,
              leg.getKey()
                  + " segment "
                  + i
                  + " passes "
                  + Math.hypot(dx, dy)
                  + " m from the charge station at "
                  + p);
          double wall = turning.get(i) ? HALF_DIAGONAL : HALF_LENGTH;
          assertTrue(
              p.getY() >= wall && p.getY() <= FIELD_WIDTH - wall,
              leg.getKey() + " segment " + i + " leaves the field at " + p);
          assertTrue(p.getX() >= GRID_X - 1e-9, leg.getKey() + " pushes into the grid at " + p);
        }
      }
    }
  }

  @Test
  void theRobotNeverTurnsInALane() {
    for (var leg : CyclePaths.allLegs().entrySet()) {
      var points = translations(leg.getValue());
      var turning = turningSegments(leg.getValue());
      for (int i = 0; i + 1 < points.size(); i++) {
        if (turning.get(i)) {
          assertTrue(
              Math.min(points.get(i).getX(), points.get(i + 1).getX()) >= LANE_EAST_X - 1e-9,
              leg.getKey() + " turns west of the lane's east end, segment " + i);
        }
      }
    }
  }

  /** For each segment, whether the heading target changes across it. */
  private static List<Boolean> turningSegments(Path path) {
    var turning = new ArrayList<Boolean>();
    double heading = Double.NaN;
    boolean changes = false;
    boolean started = false;
    for (var element : path.getPathElements()) {
      if (element instanceof Waypoint w) {
        double target = w.rotationTarget().rotation().getRadians();
        if (started) {
          turning.add(changes || !sameHeading(target, heading));
        }
        heading = target;
        changes = false;
        started = true;
      } else if (element instanceof TranslationTarget) {
        turning.add(changes);
        changes = false;
      } else if (element instanceof RotationTarget r) {
        double target = r.rotation().getRadians();
        changes |= !sameHeading(target, heading);
        heading = target;
      }
    }
    return turning;
  }

  private static boolean sameHeading(double a, double b) {
    return Math.abs(Math.IEEEremainder(a - b, 2 * Math.PI)) < 1e-6;
  }

  @Test
  void mechanismsOnlyExtendNearTheEndOfALeg() {
    for (var leg : CyclePaths.allLegs().entrySet()) {
      String name = leg.getKey();
      var events = eventDistancesFromEnd(leg.getValue());
      if (name.startsWith("gridToPiece") || name.startsWith("pieceToPiece")) {
        assertEquals(1, events.size(), name);
        assertEquals(
            Math.min(CyclePaths.INTAKE_LOWER_REMAINING_METERS, totalLength(leg.getValue())),
            events.get(0),
            0.01,
            name + " lowers the intake");
      } else if (name.startsWith("laneToNode")) {
        assertEquals(1, events.size(), name);
        assertEquals(
            CyclePaths.SCORE_RAISE_REMAINING_METERS,
            events.get(0),
            0.01,
            name + " raises to score");
      } else {
        assertEquals(0, events.size(), name + " moves with the mechanisms retracted");
      }
    }
  }

  @Test
  void lanesAreChosenByWhichSideTheTripIsOn() {
    assertEquals(UPPER_LANE_Y, laneBetween(nodeY(8), stagedPieceY(4)));
    assertEquals(LOWER_LANE_Y, laneBetween(nodeY(2), stagedPieceY(1)));
  }

  private static List<Translation2d> translations(Path path) {
    var points = new ArrayList<Translation2d>();
    for (var element : path.getPathElements()) {
      if (element instanceof Waypoint w) {
        points.add(w.translationTarget().translation());
      } else if (element instanceof TranslationTarget t) {
        points.add(t.translation());
      }
    }
    return points;
  }

  private static double totalLength(Path path) {
    var points = translations(path);
    double total = 0;
    for (int i = 0; i + 1 < points.size(); i++) {
      total += points.get(i).getDistance(points.get(i + 1));
    }
    return total;
  }

  /** Each event's distance before the end of the path, along the path. */
  private static List<Double> eventDistancesFromEnd(Path path) {
    var distances = new ArrayList<Double>();
    double total = totalLength(path);
    double traveled = 0;
    Translation2d previous = null;
    var pendingEvents = new ArrayList<Double>();
    for (var element : path.getPathElements()) {
      Translation2d point = null;
      if (element instanceof Waypoint w) {
        point = w.translationTarget().translation();
      } else if (element instanceof TranslationTarget t) {
        point = t.translation();
      } else if (element instanceof EventTrigger e) {
        pendingEvents.add(e.t_ratio());
      }
      if (point != null) {
        if (previous != null) {
          double length = previous.getDistance(point);
          for (double t : pendingEvents) {
            distances.add(total - (traveled + t * length));
          }
          pendingEvents.clear();
          traveled += length;
        }
        previous = point;
      }
    }
    return distances;
  }
}
