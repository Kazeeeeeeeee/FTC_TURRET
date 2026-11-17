package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

/**
 * EfficiencyTable
 *
 * Distance-dependent flywheel efficiency table using linear interpolation
 * and end-point clamping. Loads/saves as a simple CSV file (distance,efficiency).
 */
public class EfficiencyTable {

    public static class Point {
        public double distanceInches;
        public double efficiency;

        public Point(double distanceInches, double efficiency) {
            this.distanceInches = distanceInches;
            this.efficiency = efficiency;
        }
    }

    private final List<Point> points = new ArrayList<>();
    private static final String FILE_NAME = "turret_efficiency_table.csv";

    public EfficiencyTable() {
        // Default table has a single point equal to 0.65
        points.add(new Point(0.0, 0.65));
    }

    /**
     * Calculates the effective efficiency at a given distance using linear interpolation.
     */
    public double effectiveEfficiency(double distanceInches) {
        if (points.isEmpty()) {
            return 0.65;
        }
        if (points.size() == 1) {
            return points.get(0).efficiency;
        }

        // Clamp below lowest distance
        if (distanceInches <= points.get(0).distanceInches) {
            return points.get(0).efficiency;
        }

        // Clamp above highest distance
        if (distanceInches >= points.get(points.size() - 1).distanceInches) {
            return points.get(points.size() - 1).efficiency;
        }

        // Linear interpolation between adjacent points
        for (int i = 0; i < points.size() - 1; i++) {
            Point p1 = points.get(i);
            Point p2 = points.get(i + 1);

            if (distanceInches >= p1.distanceInches && distanceInches <= p2.distanceInches) {
                double t = (distanceInches - p1.distanceInches) / (p2.distanceInches - p1.distanceInches);
                return p1.efficiency + t * (p2.efficiency - p1.efficiency);
            }
        }

        return points.get(0).efficiency;
    }

    /**
     * Inserts or merges a (distance, efficiency) point into the table.
     * Merges points within 2.0 inches of distance.
     */
    public void insertOrMergePoint(double distanceInches, double efficiency) {
        double clampedEff = Math.min(1.0, Math.max(0.1, efficiency));

        for (Point p : points) {
            if (Math.abs(p.distanceInches - distanceInches) <= 2.0) {
                // Merge into existing point within 2 inches
                p.efficiency = clampedEff;
                sortPoints();
                return;
            }
        }

        // Add new point and sort
        points.add(new Point(distanceInches, clampedEff));
        sortPoints();
    }

    /** Deletes the last point in the table (keeping at least 1 default point) */
    public boolean removeLastPoint() {
        if (points.size() > 1) {
            points.remove(points.size() - 1);
            return true;
        }
        return false;
    }

    public List<Point> getPoints() {
        return Collections.unmodifiableList(points);
    }

    public int size() {
        return points.size();
    }

    private void sortPoints() {
        points.sort(Comparator.comparingDouble(p -> p.distanceInches));
    }

    /** Saves table to CSV file in SDK settings directory */
    public void saveToFile() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            StringBuilder sb = new StringBuilder();
            for (Point p : points) {
                sb.append(String.format(Locale.US, "%.2f,%.4f\n", p.distanceInches, p.efficiency));
            }
            ReadWriteFile.writeFile(file, sb.toString());
        } catch (Exception ignored) {
        }
    }

    /** Loads table from CSV file in SDK settings directory */
    public void loadFromFile() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (file.exists()) {
                String content = ReadWriteFile.readFile(file);
                String[] lines = content.split("\n");
                List<Point> loadedPoints = new ArrayList<>();
                for (String rawLine : lines) {
                    String lineStr = rawLine.trim();
                    if (!lineStr.isEmpty()) {
                        String[] parts = lineStr.split(",");
                        if (parts.length >= 2) {
                            double dist = Double.parseDouble(parts[0].trim());
                            double eff = Double.parseDouble(parts[1].trim());
                            loadedPoints.add(new Point(dist, eff));
                        }
                    }
                }
                if (!loadedPoints.isEmpty()) {
                    points.clear();
                    points.addAll(loadedPoints);
                    sortPoints();
                }
            }
        } catch (Exception ignored) {
        }
    }
}
