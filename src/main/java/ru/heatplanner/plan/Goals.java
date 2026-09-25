package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.linearref.LengthIndexedLine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Возможные места врезки в существующую сеть для одного старта.
 *
 * Подключение только через камеру: в существующую (если к ней примыкает менее четырёх участков) или в новую, которая
 * создаётся в выбранной точке существующего участка. Точек на участке бесконечно много, поэтому берётся ближайшая к
 * старту и точки с шагом {@link #STEP_M} на участках в пределах разумного расстояния; лучшее место выбирает поиск.
 */
public final class Goals {

    /** Вид врезки. */
    public enum Kind {
        EXISTING_CHAMBER,
        NEW_CHAMBER
    }

    /** Полезная нагрузка цели: куда именно врезка. */
    public static final class Attach {
        public final Kind kind;
        public final PlanInput.Chamber chamber;
        public final PlanInput.Segment segment;
        /** ДУ существующей сети в месте врезки (для стоимости новой камеры), мм. */
        public final int existingDn;

        Attach(Kind kind, PlanInput.Chamber chamber, PlanInput.Segment segment, int existingDn) {
            this.kind = kind;
            this.chamber = chamber;
            this.segment = segment;
            this.existingDn = existingDn;
        }
    }

    /** Шаг точек врезки вдоль участка, м. */
    private static final double STEP_M = 40;
    /** Не более такого числа целей на один поиск. */
    private static final int MAX_GOALS = 60;
    /** Участок рассматривается, если он не дальше ближайшего более чем на столько (м) плюс двукратное расстояние до него. */
    private static final double SLACK_M = 150;

    private Goals() {
    }

    /** Число линейных участков, примыкающих к камере: конец участка даёт 1, проходящая через камеру линия — 2. */
    static int adjacency(PlanInput in, PlanInput.Chamber ch) {
        int n = 0;
        for (PlanInput.Segment s : in.segments) {
            if (s.line.distance(new org.locationtech.jts.geom.GeometryFactory().createPoint(ch.xy)) > 0.3) {
                continue;
            }
            Coordinate a = s.line.getCoordinateN(0);
            Coordinate b = s.line.getCoordinateN(s.line.getNumPoints() - 1);
            n += (a.distance(ch.xy) < 0.3 || b.distance(ch.xy) < 0.3) ? 1 : 2;
        }
        return n;
    }

    public static List<Goal> forStart(PlanInput in, ObstacleSet obstacles, Coordinate origin, int dn) {
        List<Goal> out = new ArrayList<>();
        for (PlanInput.Chamber ch : in.chambers) {
            if (adjacency(in, ch) + 1 > Rules.MAX_CHAMBER_SEGMENTS) {
                continue;
            }
            Set<Obstacle> touching = new HashSet<>();
            int existing = 0;
            for (PlanInput.Segment s : in.segments) {
                if (s.line.distance(new org.locationtech.jts.geom.GeometryFactory().createPoint(ch.xy)) <= 0.3) {
                    Obstacle o = obstacleOf(obstacles, s);
                    if (o != null) {
                        touching.add(o);
                    }
                    existing = Math.max(existing, s.dn);
                }
            }
            if (!obstacles.pointBlocked(ch.xy, dn, touching)) {
                out.add(new Goal(ch.xy, touching, null, new Attach(Kind.EXISTING_CHAMBER, ch, null, existing)));
            }
        }
        // Точки на участках: ближайший участок задаёт радиус, дальше всё, что не слишком далеко.
        double nearest = Double.MAX_VALUE;
        for (PlanInput.Segment s : in.segments) {
            nearest = Math.min(nearest, s.line.distance(new org.locationtech.jts.geom.GeometryFactory().createPoint(origin)));
        }
        double reach = nearest * 2 + SLACK_M;
        List<Goal> onLines = new ArrayList<>();
        for (PlanInput.Segment s : in.segments) {
            if (s.line.distance(new org.locationtech.jts.geom.GeometryFactory().createPoint(origin)) > reach) {
                continue;
            }
            Obstacle o = obstacleOf(obstacles, s);
            if (o == null) {
                continue;
            }
            LengthIndexedLine lil = new LengthIndexedLine(s.line);
            double len = s.line.getLength();
            List<Double> at = new ArrayList<>();
            at.add(lil.project(origin));
            for (double d = STEP_M / 2; d < len; d += STEP_M) {
                at.add(d);
            }
            for (double d : at) {
                d = Math.max(0.5, Math.min(len - 0.5, d));
                Coordinate p = lil.extractPoint(d);
                if (nearChamber(in, p) || obstacles.pointBlocked(p, dn, Collections.singleton(o))) {
                    continue;
                }
                Coordinate q = lil.extractPoint(Math.min(len, d + 0.5));
                Coordinate r = lil.extractPoint(Math.max(0, d - 0.5));
                double[] dir = Angles.unit(r, q);
                onLines.add(new Goal(p, Collections.singleton(o), dir, new Attach(Kind.NEW_CHAMBER, null, s, s.dn)));
            }
        }
        onLines.sort((a, b) -> Double.compare(a.xy.distance(origin), b.xy.distance(origin)));
        out.addAll(onLines.subList(0, Math.min(onLines.size(), MAX_GOALS)));
        return out;
    }

    /** Точка ближе 10 м к существующей камере: врезка здесь идёт в неё, а не в новую (иначе число участков не сойдётся). */
    private static boolean nearChamber(PlanInput in, Coordinate p) {
        for (PlanInput.Chamber ch : in.chambers) {
            if (ch.xy.distance(p) <= Rules.CHAMBER_SNAP_M) {
                return true;
            }
        }
        return false;
    }

    private static Obstacle obstacleOf(ObstacleSet obstacles, PlanInput.Segment s) {
        for (Obstacle o : obstacles.all()) {
            if (o.isExistingNetwork() && o.id.equals(s.id)) {
                return o;
            }
        }
        return null;
    }
}