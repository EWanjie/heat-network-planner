package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineSegment;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Старт поиска для одной цели.
 *
 * Если точка подключения лежит внутри полигона ОКС (restriction_type = oks), допускается один финальный прямой участок
 * от границы полигона до самой точки, и отступ до собственного полигона на него не распространяется (разъяснение 3,
 * раздел 2.2 приложения). Участок продолжается наружу через зону отступа: поиск начинается там, где ось выходит
 * из зоны отступа собственного полигона, а участок origin — точка идёт в составе трассы как заключительный.
 * Ближайших границ может быть несколько: каждая даёт отдельный старт.
 */
public final class Start {

    /** Откуда идёт поиск. */
    public final Coordinate origin;
    /** Единичное направление движения из origin (продолжение финального участка) или null, если ограничения нет. */
    public final double[] direction;
    /** Собственные полигоны: отступ до них не проверяется. */
    public final Set<Obstacle> exempt;
    /** Сама точка подключения, если трасса начинается финальным прямым участком origin — terminalPoint. */
    public final Coordinate terminalPoint;
    public final PlanInput.Target target;
    /** Подход к ближайшей границе здания (false — запасной подход к другой границе, о чём сообщается в результате). */
    public final boolean nearestBoundary;

    Start(Coordinate origin, double[] direction, Set<Obstacle> exempt, Coordinate terminalPoint, PlanInput.Target target) {
        this(origin, direction, exempt, terminalPoint, target, true);
    }

    Start(Coordinate origin, double[] direction, Set<Obstacle> exempt, Coordinate terminalPoint, PlanInput.Target target,
          boolean nearestBoundary) {
        this.origin = origin;
        this.direction = direction;
        this.exempt = exempt;
        this.terminalPoint = terminalPoint;
        this.target = target;
        this.nearestBoundary = nearestBoundary;
    }

    private static final GeometryFactory F = new GeometryFactory();
    /** Сколько ближайших границ рассматривать, чтобы не подменять подход произвольным входом с дальней стороны. */
    private static final int MAX_APPROACHES = 3;
    /** Различными считаются подходы, направления которых отличаются не менее чем на столько градусов. */
    private static final double DISTINCT_DEG = 10;
    /** Границы на расстоянии в пределах этого допуска от самой ближайшей считаются равно ближайшими, м. */
    private static final double NEAREST_TOLERANCE_M = 0.25;

    /** Возможные старты для цели; пусто, если подойти к точке допустимым финальным участком нельзя. */
    public static List<Start> forTarget(PlanInput.Target target, ObstacleSet obstacles, int dn, RuleSet rules) {
        Point cp = F.createPoint(target.xy);
        Set<Obstacle> own = new HashSet<>();
        for (Obstacle o : obstacles.all()) {
            if ("oks".equals(o.rule.type) && o.geometry.getDimension() == 2 && o.geometry.covers(cp)) {
                own.add(o);
            }
        }
        List<Start> out = new ArrayList<>();
        if (own.isEmpty()) {
            if (!obstacles.pointBlocked(target.xy, dn, null)) {
                out.add(new Start(target.xy, null, Collections.emptySet(), null, target));
            }
            return out;
        }
        // Кандидаты: ближайшие точки каждого отрезка границы собственных полигонов; из них берутся ближайшие,
        // различающиеся направлением (точка в центре здания одинаково близка ко всем его сторонам).
        List<Coordinate[]> nearest = new ArrayList<>();
        for (Obstacle o : own) {
            for (Geometry ring : boundaryParts(o.geometry)) {
                Coordinate[] cs = ring.getCoordinates();
                for (int i = 1; i < cs.length; i++) {
                    Coordinate closest = new LineSegment(cs[i - 1], cs[i]).closestPoint(target.xy);
                    nearest.add(new Coordinate[]{closest, target.xy});
                }
            }
        }
        nearest.sort((a, b) -> Double.compare(a[0].distance(target.xy), b[0].distance(target.xy)));
        List<double[]> used = new ArrayList<>();
        double nearestDistance = nearest.isEmpty() ? 0 : nearest.get(0)[0].distance(target.xy);
        for (Coordinate[] pair : nearest) {
            if (out.size() >= MAX_APPROACHES) {
                break;
            }
            Coordinate boundary = pair[0];
            boolean isNearest = boundary.distance(target.xy) <= nearestDistance + NEAREST_TOLERANCE_M;
            if (!isNearest && rules.ownApproach == RuleSet.OwnApproachMode.NEAREST_ONLY) {
                break;
            }
            double[] u = boundary.distance(target.xy) < 1e-6 ? outwardNormal(own, boundary) : Angles.unit(target.xy, boundary);
            if (u == null) {
                continue;
            }
            boolean distinct = true;
            for (double[] prev : used) {
                if (Angles.turnDeg(prev, u) < DISTINCT_DEG) {
                    distinct = false;
                }
            }
            if (!distinct) {
                continue;
            }
            Coordinate exit = exitOfOwnZone(boundary, u, own, obstacles, dn);
            if (exit == null) {
                continue;
            }
            // Выход обязан быть вне собственных зон: иначе луч из внутреннего двора идёт обратно в тело здания.
            boolean insideOwnZone = false;
            for (Obstacle o : own) {
                insideOwnZone |= obstacles.zone(o, dn).geometry.intersects(F.createPoint(exit));
            }
            // Заключительный прямой участок от выхода из зоны до точки подключения: остальные ограничения действуют.
            if (insideOwnZone || obstacles.pointBlocked(exit, dn, own) || !obstacles.segmentClear(exit, target.xy, dn, own)) {
                continue;
            }
            used.add(u);
            out.add(new Start(exit, u, own, target.xy, target, isNearest));
        }
        return out;
    }

    private static List<Geometry> boundaryParts(Geometry area) {
        List<Geometry> parts = new ArrayList<>();
        for (int i = 0; i < area.getNumGeometries(); i++) {
            org.locationtech.jts.geom.Polygon p = (org.locationtech.jts.geom.Polygon) area.getGeometryN(i);
            parts.add(p.getExteriorRing());
            for (int h = 0; h < p.getNumInteriorRing(); h++) {
                parts.add(p.getInteriorRingN(h));
            }
        }
        return parts;
    }

    /** Внешняя нормаль к границе в точке (для точки подключения, лежащей ровно на границе): наружу из полигонов. */
    private static double[] outwardNormal(Set<Obstacle> own, Coordinate boundary) {
        for (int deg = 0; deg < 360; deg += 15) {
            double a = Math.toRadians(deg);
            Coordinate probe = new Coordinate(boundary.x + 0.05 * Math.cos(a), boundary.y + 0.05 * Math.sin(a));
            boolean inside = false;
            for (Obstacle o : own) {
                inside |= o.geometry.covers(F.createPoint(probe));
            }
            if (!inside) {
                return new double[]{Math.cos(a), Math.sin(a)};
            }
        }
        return null;
    }

    /**
     * Точка на луче от границы наружу, где ось выходит из зоны отступа собственного полигона (чуть за ней).
     * Зона считается для ДУ dn по тем же правилам, что и при проверках; вычислительный запас уже в ней.
     */
    private static Coordinate exitOfOwnZone(Coordinate boundary, double[] u, Set<Obstacle> own, ObstacleSet obstacles, int dn) {
        double reach = 0;
        for (Obstacle o : own) {
            reach = Math.max(reach, o.axisDistance(dn));
        }
        LineString ray = F.createLineString(new Coordinate[]{boundary,
                new Coordinate(boundary.x + u[0] * (reach + 20), boundary.y + u[1] * (reach + 20))});
        double exit = 0;
        for (Obstacle o : own) {
            Geometry inside = obstacles.zone(o, dn).geometry.intersection(ray);
            for (Coordinate c : inside.getCoordinates()) {
                exit = Math.max(exit, boundary.distance(c));
            }
        }
        return new Coordinate(boundary.x + u[0] * (exit + 0.02), boundary.y + u[1] * (exit + 0.02));
    }
}
