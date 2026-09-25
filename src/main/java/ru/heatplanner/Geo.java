package ru.heatplanner;

import java.util.ArrayList;
import java.util.List;

/** Плоская геометрия в метрах (EPSG:32637): расстояния, пересечения, работа с ломаными. Полилиния — double[][] {x, y}. */
public final class Geo {

    private Geo() {
    }

    static double dist(double[] a, double[] b) {
        return Math.hypot(a[0] - b[0], a[1] - b[1]);
    }

    static double length(double[][] pts) {
        double sum = 0;
        for (int i = 1; i < pts.length; i++) {
            sum += dist(pts[i - 1], pts[i]);
        }
        return sum;
    }

    /** Расстояние от точки до отрезка ab. */
    static double pointSegment(double px, double py, double[] a, double[] b) {
        double dx = b[0] - a[0];
        double dy = b[1] - a[1];
        double len2 = dx * dx + dy * dy;
        double t = len2 == 0 ? 0 : ((px - a[0]) * dx + (py - a[1]) * dy) / len2;
        t = Math.max(0, Math.min(1, t));
        return Math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy));
    }

    private static double cross(double ax, double ay, double bx, double by) {
        return ax * by - ay * bx;
    }

    /**
     * Пересечение отрезков ab и cd. Возвращает {t, u} — доли вдоль ab и cd, или null, если отрезки
     * не пересекаются (в том числе параллельны).
     */
    static double[] intersect(double[] a, double[] b, double[] c, double[] d) {
        double rx = b[0] - a[0];
        double ry = b[1] - a[1];
        double sx = d[0] - c[0];
        double sy = d[1] - c[1];
        double den = cross(rx, ry, sx, sy);
        if (Math.abs(den) < 1e-12) {
            return null;
        }
        double t = cross(c[0] - a[0], c[1] - a[1], sx, sy) / den;
        double u = cross(c[0] - a[0], c[1] - a[1], rx, ry) / den;
        if (t < 0 || t > 1 || u < 0 || u > 1) {
            return null;
        }
        return new double[]{t, u};
    }

    /** Расстояние между отрезками ab и cd (0, если пересекаются). */
    static double segmentSegment(double[] a, double[] b, double[] c, double[] d) {
        if (intersect(a, b, c, d) != null) {
            return 0;
        }
        return Math.min(Math.min(pointSegment(a[0], a[1], c, d), pointSegment(b[0], b[1], c, d)),
                Math.min(pointSegment(c[0], c[1], a, b), pointSegment(d[0], d[1], a, b)));
    }

    /** Точка внутри набора колец по правилу чётности (дырки учитываются автоматически). */
    static boolean inside(double x, double y, double[][][] rings) {
        boolean in = false;
        for (double[][] ring : rings) {
            for (int i = 0, j = ring.length - 1; i < ring.length; j = i++) {
                if ((ring[i][1] > y) != (ring[j][1] > y)
                        && x < (ring[j][0] - ring[i][0]) * (y - ring[i][1]) / (ring[j][1] - ring[i][1]) + ring[i][0]) {
                    in = !in;
                }
            }
        }
        return in;
    }

    /** Расстояние от точки до ближайшей стороны колец/линий. */
    static double pointBoundary(double x, double y, double[][][] rings) {
        double best = Double.MAX_VALUE;
        for (double[][] ring : rings) {
            for (int i = 1; i < ring.length; i++) {
                best = Math.min(best, pointSegment(x, y, ring[i - 1], ring[i]));
            }
        }
        return best;
    }

    /** Точка на ломаной на расстоянии d от её начала (d обрезается по длине). */
    static double[] pointAt(double[][] pts, double d) {
        if (d <= 0) {
            return pts[0].clone();
        }
        double acc = 0;
        for (int i = 1; i < pts.length; i++) {
            double seg = dist(pts[i - 1], pts[i]);
            if (acc + seg >= d) {
                double t = seg == 0 ? 0 : (d - acc) / seg;
                return new double[]{pts[i - 1][0] + t * (pts[i][0] - pts[i - 1][0]),
                        pts[i - 1][1] + t * (pts[i][1] - pts[i - 1][1])};
            }
            acc += seg;
        }
        return pts[pts.length - 1].clone();
    }

    /** Часть ломаной между расстояниями d0 и d1 от начала. */
    static double[][] sub(double[][] pts, double d0, double d1) {
        List<double[]> out = new ArrayList<>();
        out.add(pointAt(pts, d0));
        double acc = 0;
        for (int i = 1; i < pts.length; i++) {
            acc += dist(pts[i - 1], pts[i]);
            if (acc > d0 + 1e-9 && acc < d1 - 1e-9) {
                out.add(pts[i].clone());
            }
        }
        out.add(pointAt(pts, d1));
        return out.toArray(new double[0][]);
    }

    /** Ближайшая к точке позиция на ломаной: {расстояние вдоль ломаной, расстояние до неё}. */
    static double[] project(double[][] pts, double px, double py) {
        double best = Double.MAX_VALUE;
        double along = 0;
        double acc = 0;
        for (int i = 1; i < pts.length; i++) {
            double[] a = pts[i - 1];
            double[] b = pts[i];
            double dx = b[0] - a[0];
            double dy = b[1] - a[1];
            double len2 = dx * dx + dy * dy;
            double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((px - a[0]) * dx + (py - a[1]) * dy) / len2));
            double d = Math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy));
            if (d < best) {
                best = d;
                along = acc + t * Math.sqrt(len2);
            }
            acc += Math.sqrt(len2);
        }
        return new double[]{along, best};
    }

    static double[][] reverse(double[][] pts) {
        double[][] out = new double[pts.length][];
        for (int i = 0; i < pts.length; i++) {
            out[i] = pts[pts.length - 1 - i];
        }
        return out;
    }

    /** Склейка двух ломаных, где конец первой совпадает с началом второй. */
    static double[][] join(double[][] first, double[][] second) {
        double[][] out = new double[first.length + second.length - 1][];
        System.arraycopy(first, 0, out, 0, first.length);
        System.arraycopy(second, 1, out, first.length, second.length - 1);
        return out;
    }

    /** Число заметных поворотов на ломаной (отклонение курса больше порога, градусы). */
    static int turns(double[][] pts, double thresholdDeg) {
        int count = 0;
        for (int i = 1; i + 1 < pts.length; i++) {
            double a1 = Math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0]);
            double a2 = Math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0]);
            double diff = Math.abs(Math.toDegrees(a2 - a1));
            diff = diff > 180 ? 360 - diff : diff;
            if (diff > thresholdDeg) {
                count++;
            }
        }
        return count;
    }
}
