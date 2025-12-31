package org.firstinspires.ftc.teamcode.subsystems;  // or pedroPathing if you prefer

import com.bylazar.configurables.annotations.Configurable;
// NO IgnoreConfigurable import here

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

@Configurable
public class Drawing {

    public static final double ROBOT_RADIUS = 9.0;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    // ===== CONFIGURABLE STARTING POSE =====
    public static double START_X = 72.0;
    public static double START_Y = 72.0;
    public static double START_HEADING_DEG = 0.0;

    // No @IgnoreConfigurable – just a normal static helper
    public static Pose getStartingPose() {
        return new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING_DEG)
        );
    }

    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    // ===== DRAW HELPERS =====

    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    public static void drawRobot(Pose pose, Style style) {
        if (pose == null) return;
        if (Double.isNaN(pose.getX()) ||
                Double.isNaN(pose.getY()) ||
                Double.isNaN(pose.getHeading())) {
            return;
        }

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading(); // radians

        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.circle(ROBOT_RADIUS);

        double dx = Math.cos(heading) * ROBOT_RADIUS;
        double dy = Math.sin(heading) * ROBOT_RADIUS;

        double x1 = x + dx / 2.0;
        double y1 = y + dy / 2.0;
        double x2 = x + dx;
        double y2 = y + dy;

        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    public static void drawPath(Path path, Style style) {
        if (path == null) return;

        double[][] points = path.getPanelsDrawingPoints();
        if (points == null || points.length < 2 || points[0].length == 0) return;

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[1][0]);
        for (int i = 1; i < points[0].length; i++) {
            panelsField.line(points[0][i], points[1][i]);
        }
    }

    public static void drawPath(PathChain pathChain, Style style) {
        if (pathChain == null) return;
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    public static void sendPacket() {
        panelsField.update();
    }
}
