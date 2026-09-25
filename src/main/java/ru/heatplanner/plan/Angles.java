package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;

/** Углы между направлениями. Направление — единичный вектор {dx, dy}. */
final class Angles {

    private Angles() {
    }

    static double[] unit(Coordinate from, Coordinate to) {
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        double len = Math.hypot(dx, dy);
        return len == 0 ? new double[]{0, 0} : new double[]{dx / len, dy / len};
    }

    /** Поворот при переходе с направления a на направление b: 0° — по прямой, 180° — разворот. */
    static double turnDeg(double[] a, double[] b) {
        double dot = Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1]));
        return Math.toDegrees(Math.acos(dot));
    }

    /** Острый угол между двумя прямыми (направления без учёта знака), 0–90°. */
    static double acuteDeg(double[] a, double[] b) {
        double dot = Math.abs(Math.max(-1, Math.min(1, a[0] * b[0] + a[1] * b[1])));
        return Math.toDegrees(Math.acos(dot));
    }
}
