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
    /**
     * Точка на границе здания (или в пределах этого расстояния, м) принадлежит зданию: при переводе долготы и широты
     * в метры прямая граница полигона и точка, лежащая на ней, расходятся на миллиметры.
     */
    static final double BOUNDARY_TOLERANCE_M = 0.05;

    /** Полигоны ОКС, содержащие точку подключения (с допуском на границу). */
    public static List<Obstacle> ownPolygons(ObstacleSet obstacles, Coordinate cp) {
        Point p = F.createPoint(cp);
        List<Obstacle> own = new ArrayList<>();
        for (Obstacle o : obstacles.all()) {
            if ("oks".equals(o.rule.type) && o.geometry.getDimension() == 2
                    && (o.geometry.covers(p) || o.geometry.distance(p) <= BOUNDARY_TOLERANCE_M)) {
                own.add(o);
            }
        }
        return own;
    }
    /** Сколько ближайших границ рассматривать, чтобы не подменять подход произвольным входом с дальней стороны. */
    private static final int MAX_APPROACHES = 3;
    /** Различными считаются подходы, направления которых отличаются не менее чем на столько градусов. */
    private static final double DISTINCT_DEG = 10;
    /** Границы на расстоянии в пределах этого допуска от самой ближайшей считаются равно ближайшими, м. */
    private static final double NEAREST_TOLERANCE_M = 0.25;

    /** Возможные старты для цели; пусто, если подойти к точке допустимым финальным участком нельзя. */
    public static List<Start> forTarget(PlanInput.Target target, ObstacleSet obstacles, int dn, RuleSet rules) {
        return forTarget(target, obstacles, dn, rules, null);
    }

    /** Причина, по которой подход к цели в рассматриваемом направлении невозможен. */
    public static final class Refusal {
        public enum Code {
            /** Точка подключения вне здания ОКС лежит в запретной зоне другого ограничения. */
            POINT_IN_OTHER_ZONE,
            /** Ближайшая граница — внутренний двор, и выход из зоны собственного здания во дворе невозможен. */
            NEAREST_BOUNDARY_IN_COURTYARD,
            /** Выход наружу из зоны собственного здания лежит в зоне отступа другого ограничения. */
            EXIT_IN_OTHER_ZONE,
            /** Заключительный прямой участок нарушает отступ до другого ограничения. */
            FINAL_SEGMENT_BLOCKED,
            /** Ближайшая стена в узкой выемке здания: прямой выход наружу упирается в другое крыло того же здания. */
            NEAREST_WALL_IN_NOTCH,
            /** Направление подхода не определено. */
            NO_DIRECTION
        }

        public final Code code;
        /** Ограничение, отступ до которого мешает (или null). */
        public final Obstacle blocker;
        /** Расстояние от точки до границы, м. */
        public final double boundaryDistance;

        Refusal(Code code, Obstacle blocker, double boundaryDistance) {
            this.code = code;
            this.blocker = blocker;
            this.boundaryDistance = boundaryDistance;
        }

        /** Причина для пользователя: почему к этому зданию строго по правилу подключиться нельзя. */
        public String message() {
            String who = blocker == null ? "" : " (" + blocker + ")";
            switch (code) {
                case POINT_IN_OTHER_ZONE:
                    return "точка подключения лежит в зоне отступа другого объекта" + who;
                case NEAREST_BOUNDARY_IN_COURTYARD:
                    return "ближайшая к точке граница здания находится во внутреннем дворе, выход из зоны отступа невозможен";
                case NEAREST_WALL_IN_NOTCH:
                    return "ближайшая стена в узкой выемке здания, прямой участок упирается в другое крыло того же здания";
                case EXIT_IN_OTHER_ZONE:
                    return "выход от ближайшей стены попадает в зону отступа другого объекта" + who;
                case FINAL_SEGMENT_BLOCKED:
                    return "финальный прямой участок нарушает отступ до другого объекта" + who;
                default:
                    return "направление подхода не определено";
            }
        }
    }

    /** То же, что forTarget, но причины отказов по рассмотренным границам добавляются в refusals (если он не null). */
    public static List<Start> forTarget(PlanInput.Target target, ObstacleSet obstacles, int dn, RuleSet rules,
                                        List<Refusal> refusals) {
        Point cp = F.createPoint(target.xy);
        Set<Obstacle> own = new HashSet<>(ownPolygons(obstacles, target.xy));
        List<Start> out = new ArrayList<>();
        if (own.isEmpty()) {
            Obstacle b = pointBlocker(target.xy, dn, obstacles, null);
            if (b == null) {
                out.add(new Start(target.xy, null, Collections.emptySet(), null, target));
            } else if (refusals != null) {
                refusals.add(new Refusal(Refusal.Code.POINT_IN_OTHER_ZONE, b, 0));
            }
            return out;
        }
        // Кандидаты: ближайшие точки каждого отрезка границы собственных полигонов; из них берутся ближайшие,
        // Кандидаты: ближайшие точки каждого отрезка границы собственных полигонов; из них берутся ближайшие,
        // различающиеся направлением (точка в центре здания одинаково близка ко всем его сторонам).
        List<Cand> nearest = new ArrayList<>();
        for (Obstacle o : own) {
            for (Ring ring : boundaryParts(o.geometry)) {
                Coordinate[] cs = ring.geometry.getCoordinates();
                for (int i = 1; i < cs.length; i++) {
                    nearest.add(new Cand(new LineSegment(cs[i - 1], cs[i]).closestPoint(target.xy), ring.hole));
                }
            }
        }
        nearest.sort((a, b) -> Double.compare(a.boundary.distance(target.xy), b.boundary.distance(target.xy)));
        List<double[]> used = new ArrayList<>();
        double nearestDistance = nearest.isEmpty() ? 0 : nearest.get(0).boundary.distance(target.xy);
        for (Cand cand : nearest) {
            if (out.size() >= MAX_APPROACHES) {
                break;
            }
            Coordinate boundary = cand.boundary;
            double dist = boundary.distance(target.xy);
            boolean isNearest = dist <= nearestDistance + NEAREST_TOLERANCE_M;
            if (!isNearest && (rules.ownApproach == RuleSet.OwnApproachMode.NEAREST_ONLY || !out.isEmpty())) {
                // Запасные подходы только для здания, к которому по правилу (к ближайшей границе) подойти нельзя.
                break;
            }
            boolean onBoundary = dist <= BOUNDARY_TOLERANCE_M;
            double[] u = onBoundary ? outwardNormal(own, boundary) : Angles.unit(target.xy, boundary);
            if (u == null) {
                refuse(refusals, isNearest, Refusal.Code.NO_DIRECTION, null, dist);
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
            // Выхода нет, если луч от границы упирается в тело здания раньше, чем выходит из зоны (узкий двор).
            if (exit == null) {
                refuse(refusals, isNearest, cand.hole ? Refusal.Code.NEAREST_BOUNDARY_IN_COURTYARD
                        : Refusal.Code.NEAREST_WALL_IN_NOTCH, null, dist);
                continue;
            }
            Obstacle atExit = pointBlocker(exit, dn, obstacles, own);
            if (atExit != null) {
                refuse(refusals, isNearest, Refusal.Code.EXIT_IN_OTHER_ZONE, atExit, dist);
                continue;
            }
            // Заключительный прямой участок от выхода из зоны до точки подключения: остальные ограничения действуют.
            Obstacle onSegment = obstacles.firstViolation(exit, target.xy, dn, own);
            if (onSegment != null) {
                refuse(refusals, isNearest, Refusal.Code.FINAL_SEGMENT_BLOCKED, onSegment, dist);
                continue;
            }
            used.add(u);
            out.add(new Start(exit, u, own, target.xy, target, isNearest));
        }
        return out;
    }

    /** Отказы записываются только для ближайших границ: по ним и решается, можно ли подойти строго по правилу. */
    private static void refuse(List<Refusal> refusals, boolean nearest, Refusal.Code code, Obstacle blocker, double dist) {
        if (refusals != null && nearest) {
            refusals.add(new Refusal(code, blocker, dist));
        }
    }

    private static Obstacle pointBlocker(Coordinate p, int dn, ObstacleSet obstacles, Set<Obstacle> exempt) {
        Point point = F.createPoint(p);
        for (Obstacle o : obstacles.near(point.getEnvelopeInternal(), dn)) {
            if ((exempt == null || !exempt.contains(o)) && obstacles.zone(o, dn).prepared.intersects(point)) {
                return o;
            }
        }
        return null;
    }

    private static final class Cand {
        final Coordinate boundary;
        final boolean hole;

        Cand(Coordinate boundary, boolean hole) {
            this.boundary = boundary;
            this.hole = hole;
        }
    }

    private static final class Ring {
        final Geometry geometry;
        final boolean hole;

        Ring(Geometry geometry, boolean hole) {
            this.geometry = geometry;
            this.hole = hole;
        }
    }

    private static List<Ring> boundaryParts(Geometry area) {
        List<Ring> parts = new ArrayList<>();
        for (int i = 0; i < area.getNumGeometries(); i++) {
            org.locationtech.jts.geom.Polygon p = (org.locationtech.jts.geom.Polygon) area.getGeometryN(i);
            parts.add(new Ring(p.getExteriorRing(), false));
            for (int h = 0; h < p.getNumInteriorRing(); h++) {
                parts.add(new Ring(p.getInteriorRingN(h), true));
            }
        }
        return parts;
    }

    /**
     * Внешняя нормаль к границе в точке (для точки подключения, лежащей на границе здания или в миллиметрах от неё):
     * перпендикуляр к ближайшему отрезку границы, направленный из полигонов наружу.
     */
    private static double[] outwardNormal(Set<Obstacle> own, Coordinate boundary) {
        LineSegment nearest = null;
        double best = Double.MAX_VALUE;
        for (Obstacle o : own) {
            for (Ring ring : boundaryParts(o.geometry)) {
                Coordinate[] cs = ring.geometry.getCoordinates();
                for (int i = 1; i < cs.length; i++) {
                    LineSegment s = new LineSegment(cs[i - 1], cs[i]);
                    double d = s.distance(boundary);
                    if (d < best && s.getLength() > 1e-9) {
                        best = d;
                        nearest = s;
                    }
                }
            }
        }
        if (nearest == null) {
            return null;
        }
        double len = nearest.getLength();
        double nx = -(nearest.p1.y - nearest.p0.y) / len;
        double ny = (nearest.p1.x - nearest.p0.x) / len;
        for (int sign : new int[]{1, -1}) {
            Coordinate probe = new Coordinate(boundary.x + sign * nx * 0.5, boundary.y + sign * ny * 0.5);
            boolean inside = false;
            for (Obstacle o : own) {
                inside |= o.geometry.covers(F.createPoint(probe));
            }
            if (!inside) {
                return new double[]{sign * nx, sign * ny};
            }
        }
        return null;
    }
    /**
     * Точка на луче от границы наружу, где ось выходит из зоны отступа собственного полигона (чуть за ней).
     * Зона считается для ДУ dn по тем же правилам, что и при проверках; вычислительный запас уже в ней.
     * Луч идёт от границы непрерывно: если раньше выхода из зоны он входит в тело собственного здания (ближайшая
     * граница — двор, за которым снова здание), выхода наружу нет и возвращается null.
     */
    private static Coordinate exitOfOwnZone(Coordinate boundary, double[] u, Set<Obstacle> own, ObstacleSet obstacles, int dn) {
        double reach = 0;
        for (Obstacle o : own) {
            reach = Math.max(reach, o.axisDistance(obstacles.zoneDn(dn)));
        }
        final double step = 0.05;
        for (double s = step; s <= reach + 20; s += step) {
            Coordinate p = new Coordinate(boundary.x + u[0] * s, boundary.y + u[1] * s);
            Point point = F.createPoint(p);
            boolean inZone = false;
            for (Obstacle o : own) {
                if (o.geometry.covers(point)) {
                    return null;
                }
                inZone |= obstacles.zone(o, dn).prepared.intersects(point);
            }
            if (!inZone) {
                return p;
            }
        }
        return null;
    }
}