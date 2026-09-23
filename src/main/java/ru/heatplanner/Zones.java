package ru.heatplanner;

import java.util.ArrayList;
import java.util.List;

/**
 * Зоны специального прохода вдоль трубы (раздел 5 технического приложения): где труба пересекает дорогу,
 * трамвайные пути, газопровод, кабель или существующую теплосеть. Считается по настоящей геометрии, не по ячейкам.
 *
 * Полигон (дорога, трамвай): участок от входа в полигон до выхода плюс extent (3 м) с каждой стороны.
 * Линия (газ, кабель, теплосеть): по extent (2 м) в обе стороны от точки пересечения.
 */
final class Zones {

    /** Допуск, м: касание в самом начале или конце трубы (врезка, точка ОКС) не считается пересечением. */
    private static final double END_TOLERANCE = 0.05;

    private Zones() {
    }

    /** Участок трубы с особым коэффициентом; from/to — расстояния от начала ломаной. */
    static final class Zone {
        final double from;
        final double to;
        final double k;
        final Rules.Rule rule;
        /** Наименьший угол пересечения с границей полигона, градусы; 90, если не применимо. */
        final double angleDeg;

        Zone(double from, double to, double k, Rules.Rule rule, double angleDeg) {
            this.from = from;
            this.to = to;
            this.k = k;
            this.rule = rule;
            this.angleDeg = angleDeg;
        }
    }

    /** Непересекающиеся зоны вдоль ломаной в порядке возрастания; между ними — обычная прокладка. */
    static List<Zone> of(double[][] pts, PlanModel model) {
        double length = Geo.length(pts);
        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
        for (double[] p : pts) {
            minX = Math.min(minX, p[0]);
            minY = Math.min(minY, p[1]);
            maxX = Math.max(maxX, p[0]);
            maxY = Math.max(maxY, p[1]);
        }
        List<Zone> raw = new ArrayList<>();
        for (PlanModel.Shape shape : model.shapes) {
            Rules.Rule rule = shape.rule;
            if (rule.forbidden) {
                continue;
            }
            double pad = rule.extent + 1;
            if (shape.maxX + pad < minX || shape.minX - pad > maxX || shape.maxY + pad < minY || shape.minY - pad > maxY) {
                continue;
            }
            if (shape.area) {
                areaZones(pts, length, shape, raw);
            } else {
                lineZones(pts, length, shape, raw);
            }
        }
        return flatten(raw);
    }

    private static void areaZones(double[][] pts, double length, PlanModel.Shape shape, List<Zone> out) {
        // Расстояния вдоль трубы, где она пересекает границу полигона, и угол в каждом пересечении.
        List<double[]> crossings = new ArrayList<>();
        double base = 0;
        for (int i = 1; i < pts.length; i++) {
            double segLen = Geo.dist(pts[i - 1], pts[i]);
            for (double[][] ring : shape.rings) {
                for (int j = 1; j < ring.length; j++) {
                    double[] hit = Geo.intersect(pts[i - 1], pts[i], ring[j - 1], ring[j]);
                    if (hit != null) {
                        crossings.add(new double[]{base + hit[0] * segLen, angle(pts[i - 1], pts[i], ring[j - 1], ring[j])});
                    }
                }
            }
            base += segLen;
        }
        crossings.sort((p, q) -> Double.compare(p[0], q[0]));
        boolean in = Geo.inside(pts[0][0], pts[0][1], shape.rings);
        double enter = 0;
        double minAngle = 90;
        for (double[] c : crossings) {
            minAngle = Math.min(minAngle, c[1]);
            if (!in) {
                enter = c[0];
                in = true;
            } else {
                out.add(zone(enter, c[0], length, shape.rule, minAngle));
                in = false;
                minAngle = 90;
            }
        }
        if (in) {
            out.add(zone(enter, length, length, shape.rule, minAngle));
        }
    }

    private static Zone zone(double from, double to, double length, Rules.Rule rule, double angle) {
        return new Zone(Math.max(0, from - rule.extent), Math.min(length, to + rule.extent), rule.kSpec, rule, angle);
    }

    private static void lineZones(double[][] pts, double length, PlanModel.Shape shape, List<Zone> out) {
        double base = 0;
        for (int i = 1; i < pts.length; i++) {
            double segLen = Geo.dist(pts[i - 1], pts[i]);
            for (double[][] line : shape.rings) {
                for (int j = 1; j < line.length; j++) {
                    double[] hit = Geo.intersect(pts[i - 1], pts[i], line[j - 1], line[j]);
                    if (hit == null) {
                        continue;
                    }
                    double d = base + hit[0] * segLen;
                    if (d < END_TOLERANCE || d > length - END_TOLERANCE) {
                        continue;
                    }
                    out.add(new Zone(Math.max(0, d - shape.rule.extent), Math.min(length, d + shape.rule.extent),
                            shape.rule.kSpec, shape.rule, 90));
                }
            }
            base += segLen;
        }
    }

    /** Острый угол между двумя отрезками, градусы. */
    private static double angle(double[] a, double[] b, double[] c, double[] d) {
        double a1 = Math.atan2(b[1] - a[1], b[0] - a[0]);
        double a2 = Math.atan2(d[1] - c[1], d[0] - c[0]);
        double diff = Math.abs(Math.toDegrees(a1 - a2)) % 180;
        return diff > 90 ? 180 - diff : diff;
    }

    /** Накладывающиеся зоны превращаются в непересекающиеся: на перекрытии берётся наибольший коэффициент. */
    private static List<Zone> flatten(List<Zone> raw) {
        if (raw.isEmpty()) {
            return raw;
        }
        List<Double> cuts = new ArrayList<>();
        for (Zone z : raw) {
            cuts.add(z.from);
            cuts.add(z.to);
        }
        cuts.sort(null);
        List<Zone> out = new ArrayList<>();
        for (int i = 1; i < cuts.size(); i++) {
            double a = cuts.get(i - 1);
            double b = cuts.get(i);
            if (b - a < 1e-6) {
                continue;
            }
            Zone best = null;
            for (Zone z : raw) {
                if (z.from <= a + 1e-9 && z.to >= b - 1e-9 && (best == null || z.k > best.k)) {
                    best = z;
                }
            }
            if (best == null) {
                continue;
            }
            Zone last = out.isEmpty() ? null : out.get(out.size() - 1);
            if (last != null && Math.abs(last.to - a) < 1e-6 && last.rule == best.rule && last.k == best.k) {
                out.set(out.size() - 1, new Zone(last.from, b, best.k, best.rule, Math.min(last.angleDeg, best.angleDeg)));
            } else {
                out.add(new Zone(a, b, best.k, best.rule, best.angleDeg));
            }
        }
        return out;
    }
}
