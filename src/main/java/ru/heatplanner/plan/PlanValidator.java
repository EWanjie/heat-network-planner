package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineSegment;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;

import java.util.ArrayList;
import java.util.List;

/**
 * Независимая проверка готовых трасс по правилам приложения.
 *
 * Валидатор не пользуется графом видимости, растром и кэшем запретных зон из поиска: отступы он проверяет прямым
 * расстоянием JTS между самой геометрией ограничения и осью отрезка, а числа берёт из {@link Rules}. Поэтому ошибка
 * поиска не может незаметно повторяться в проверке.
 *
 * Нарушения делятся на две группы: RULE — нарушено правило для отдельной трассы (отступ, угол, поворот, длина ДУ,
 * непрерывность), STRUCTURE — нарушено требование к сети в целом (трассы не должны пересекаться и идти вместе
 * вне узла).
 */
public final class PlanValidator {

    public enum Group {
        RULE,
        STRUCTURE
    }

    public static final class Violation {
        public final Group group;
        public final String code;
        public final String targetId;
        public final String message;

        Violation(Group group, String code, String targetId, String message) {
            this.group = group;
            this.code = code;
            this.targetId = targetId;
            this.message = message;
        }
    }

    private static final double TOL = 1e-6;
    private static final GeometryFactory F = new GeometryFactory();

    private PlanValidator() {
    }

    public static List<Violation> validate(PlanInput in, ObstacleSet obstacles, List<Route> routes) {
        List<Violation> out = new ArrayList<>();
        for (Route r : routes) {
            checkRoute(in, obstacles, r, out);
        }
        checkStructure(routes, out);
        return out;
    }

    private static void checkRoute(PlanInput in, ObstacleSet obstacles, Route r, List<Violation> out) {
        String id = String.valueOf(r.start.target.id.value());
        List<Route.Leg> legs = r.legs;
        if (legs.isEmpty()) {
            out.add(new Violation(Group.RULE, "EMPTY_ROUTE", id, "трасса не содержит участков"));
            return;
        }
        // Непрерывность и концы.
        if (legs.get(0).from.distance(r.start.target.xy) > 0.01) {
            out.add(new Violation(Group.RULE, "NOT_FROM_TARGET", id, "трасса не начинается в точке подключения"));
        }
        for (int i = 1; i < legs.size(); i++) {
            if (legs.get(i - 1).to.distance(legs.get(i).from) > 0.01) {
                out.add(new Violation(Group.RULE, "GAP", id, "разрыв между участками " + i + " и " + (i + 1)));
            }
        }
        Route.Leg last = legs.get(legs.size() - 1);
        if (last.to.distance(r.goal.xy) > 0.01) {
            out.add(new Violation(Group.RULE, "NOT_TO_GOAL", id, "трасса не заканчивается в месте врезки"));
        }
        Object payload = r.goal.payload;
        if (payload instanceof Goals.Attach) {
            Goals.Attach a = (Goals.Attach) payload;
            Point end = F.createPoint(r.goal.xy);
            if (a.kind == Goals.Kind.NEW_CHAMBER && a.segment.line.distance(end) > 0.05) {
                out.add(new Violation(Group.RULE, "TIE_IN_OFF_NETWORK", id, "новая камера не лежит на существующем участке"));
            }
            if (a.kind == Goals.Kind.EXISTING_CHAMBER && a.chamber.xy.distance(r.goal.xy) > 0.05) {
                out.add(new Violation(Group.RULE, "TIE_IN_OFF_CHAMBER", id, "врезка не в существующей камере"));
            }
            if (in != null && a.kind == Goals.Kind.EXISTING_CHAMBER && Goals.adjacency(in, a.chamber) + 1 > Rules.MAX_CHAMBER_SEGMENTS) {
                out.add(new Violation(Group.RULE, "CHAMBER_FULL", id, "к камере примыкает больше четырёх участков"));
            }
        }
        // Первый участок — финальный прямой к точке внутри здания (если она внутри).
        if (!legs.get(0).terminal && ownPolygons(obstacles, r.start.target.xy).size() > 0) {
            out.add(new Violation(Group.RULE, "NO_TERMINAL_LEG", id, "нет финального прямого участка от границы здания до точки"));
        }
        // Повороты.
        for (int i = 1; i < legs.size(); i++) {
            double turn = turnDeg(legs.get(i - 1), legs.get(i));
            if (turn > Rules.MAX_TURN_DEG + 1e-6) {
                out.add(new Violation(Group.RULE, "TURN", id, "поворот " + Math.round(turn) + "° между участками " + i + " и " + (i + 1)));
            }
        }
        // ДУ: расход и предельная длина.
        int idx = Rules.indexOfDn(r.dn);
        if (idx < 0 || Rules.CAPACITY[idx] + 1e-9 < r.start.target.flow) {
            out.add(new Violation(Group.RULE, "CAPACITY", id, "ДУ " + r.dn + " не пропускает расход " + r.start.target.flow + " т/ч"));
        }
        double length = 0;
        for (Route.Leg l : legs) {
            length += l.from.distance(l.to);
        }
        if (idx >= 0 && length > Rules.MAX_LENGTH[idx] + 1e-6) {
            out.add(new Violation(Group.RULE, "MAX_LENGTH", id, "длина " + Math.round(length) + " м больше предельной "
                    + Math.round(Rules.MAX_LENGTH[idx]) + " м для ДУ " + r.dn));
        }
        // Отступы и специальные проходы по каждому участку.
        List<Obstacle> own = ownPolygons(obstacles, r.start.target.xy);
        for (int i = 0; i < legs.size(); i++) {
            checkLeg(obstacles, r.dn, r.goal.exempt, legs.get(i), i + 1, own, id, out);
        }
    }

    private static void checkLeg(ObstacleSet obstacles, int dn, java.util.Set<Obstacle> goalExempt, Route.Leg leg, int number,
                                 List<Obstacle> own, String id, List<Violation> out) {
        LineString line = F.createLineString(new Coordinate[]{leg.from, leg.to});
        for (Obstacle o : obstacles.all()) {
            boolean crossed = leg.special && leg.crossed.contains(o);
            boolean exempt = (leg.terminal && own.contains(o)) || (leg.tieInApproach && goalExempt.contains(o));
            if (exempt || crossed) {
                continue;
            }
            double required = o.axisDistance(dn);
            double distance = o.geometry.distance(line);
            if (distance < required - TOL) {
                out.add(new Violation(Group.RULE, "CLEARANCE", id, "участок " + number + ": до объекта " + o + " "
                        + round2(distance) + " м, требуется не менее " + round2(required) + " м"));
            }
        }
        if (!leg.special) {
            return;
        }
        double k = 1;
        for (Obstacle o : leg.crossed) {
            k = Math.max(k, o.rule.kSpec);
            if (!o.crossable()) {
                out.add(new Violation(Group.RULE, "SPECIAL_NOT_ALLOWED", id, "участок " + number + ": объект " + o + " пересекать нельзя"));
            }
            if (!o.geometry.intersects(line)) {
                out.add(new Violation(Group.RULE, "SPECIAL_NO_CROSSING", id, "участок " + number + ": специальный проход не пересекает " + o));
                continue;
            }
            double extent = o.rule.extent;
            if (o.geometry.distance(F.createPoint(leg.from)) < extent - 0.05 || o.geometry.distance(F.createPoint(leg.to)) < extent - 0.05) {
                out.add(new Violation(Group.RULE, "SPECIAL_EXTENT", id, "участок " + number + ": интервал прохода через " + o
                        + " короче требуемых " + round2(extent) + " м за границей"));
            }
            if (o.rule.minAngleDeg > 0 && o.geometry.getDimension() == 2) {
                double angle = entryAngle(o.geometry, line);
                if (angle < o.rule.minAngleDeg - 0.5) {
                    out.add(new Violation(Group.RULE, "SPECIAL_ANGLE", id, "участок " + number + ": угол пересечения " + o + " "
                            + Math.round(angle) + "°, требуется не менее " + Math.round(o.rule.minAngleDeg) + "°"));
                }
            }
        }
        if (Math.abs(leg.kSpec - k) > 1e-9) {
            out.add(new Violation(Group.RULE, "SPECIAL_K", id, "участок " + number + ": коэффициент " + leg.kSpec + ", по правилам " + k));
        }
    }

    /** Наименьший острый угол между участком и границей полигона в точках пересечения, градусы. */
    private static double entryAngle(Geometry polygon, LineString leg) {
        Coordinate a = leg.getCoordinateN(0);
        Coordinate b = leg.getCoordinateN(1);
        double legAngle = Math.atan2(b.y - a.y, b.x - a.x);
        double best = 90;
        Geometry boundary = polygon.getBoundary();
        for (Coordinate c : boundary.intersection(leg).getCoordinates()) {
            double bestDistance = Double.MAX_VALUE;
            double edgeAngle = 0;
            for (int g = 0; g < boundary.getNumGeometries(); g++) {
                Coordinate[] cs = boundary.getGeometryN(g).getCoordinates();
                for (int i = 1; i < cs.length; i++) {
                    LineSegment s = new LineSegment(cs[i - 1], cs[i]);
                    double d = s.distance(c);
                    if (d < bestDistance && s.getLength() > 1e-9) {
                        bestDistance = d;
                        edgeAngle = Math.atan2(cs[i].y - cs[i - 1].y, cs[i].x - cs[i - 1].x);
                    }
                }
            }
            double diff = Math.abs(Math.toDegrees(legAngle - edgeAngle)) % 180;
            best = Math.min(best, diff > 90 ? 180 - diff : diff);
        }
        return best;
    }

    private static List<Obstacle> ownPolygons(ObstacleSet obstacles, Coordinate cp) {
        List<Obstacle> own = new ArrayList<>();
        Point p = F.createPoint(cp);
        for (Obstacle o : obstacles.all()) {
            if ("oks".equals(o.rule.type) && o.geometry.getDimension() == 2 && o.geometry.covers(p)) {
                own.add(o);
            }
        }
        return own;
    }

    private static double turnDeg(Route.Leg a, Route.Leg b) {
        double d = Math.toDegrees(Math.atan2(b.to.y - b.from.y, b.to.x - b.from.x) - Math.atan2(a.to.y - a.from.y, a.to.x - a.from.x));
        d = Math.abs(d) % 360;
        return d > 180 ? 360 - d : d;
    }

    /** Трассы не должны пересекаться и идти вместе вне общего узла: длина общих участков и число точек пересечения. */
    private static void checkStructure(List<Route> routes, List<Violation> out) {
        List<LineString> lines = new ArrayList<>();
        for (Route r : routes) {
            List<Coordinate> cs = new ArrayList<>();
            for (Coordinate c : r.vertices()) {
                cs.add(c);
            }
            lines.add(F.createLineString(cs.toArray(new Coordinate[0])));
        }
        for (int i = 0; i < routes.size(); i++) {
            for (int j = i + 1; j < routes.size(); j++) {
                Geometry common = lines.get(i).intersection(lines.get(j));
                if (common.isEmpty()) {
                    continue;
                }
                // Общий конец у существующей сети (одна и та же точка врезки) допустим.
                if (common.getLength() < 0.05 && routes.get(i).goal.xy.distance(routes.get(j).goal.xy) < 0.05) {
                    continue;
                }
                String a = String.valueOf(routes.get(i).start.target.id.value());
                String b = String.valueOf(routes.get(j).start.target.id.value());
                String what = common.getLength() >= 0.05 ? "идут вместе " + Math.round(common.getLength()) + " м" : "пересекаются";
                out.add(new Violation(Group.STRUCTURE, "ROUTES_OVERLAP", a, "трассы точек " + a + " и " + b + " " + what
                        + ", а не образуют дерево с ветвлением в камере"));
            }
        }
    }

    // ---- Дерево ветвей ------------------------------------------------------------------------------------------------

    /**
     * Проверка всей сети: правила по каждой ветви на итоговых диаметрах участков, расход и диаметры участков, предельная
     * длина одного диаметра на пути к сети, узлы примыкания и отсутствие пересечений вне узлов.
     * Расходы пересчитываются здесь заново по структуре ветвей, а не берутся у оценки.
     */
    public static List<Violation> validateTree(PlanInput in, ObstacleSet obstacles, List<Branch> bs, TreeEvaluator.Result ev) {
        List<Violation> out = new ArrayList<>();
        int n = bs.size();
        double[] sub = new double[n];
        for (int i = 0; i < n; i++) {
            sub[i] = bs.get(i).target.flow;
        }
        for (int i = n - 1; i >= 0; i--) {
            if (bs.get(i).parent >= 0) {
                sub[bs.get(i).parent] += sub[i];
            }
        }
        List<List<TreeEvaluator.Piece>> pieces = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            pieces.add(new ArrayList<>());
        }
        for (TreeEvaluator.Piece p : ev.pieces) {
            pieces.get(p.branch).add(p);
        }
        for (int b = 0; b < n; b++) {
            Branch br = bs.get(b);
            String id = String.valueOf(br.target.id.value());
            List<Route.Leg> legs = br.route.legs;
            if (legs.get(0).from.distance(br.target.xy) > 0.01) {
                out.add(new Violation(Group.RULE, "NOT_FROM_TARGET", id, "трасса не начинается в точке подключения"));
            }
            for (int i = 1; i < legs.size(); i++) {
                if (legs.get(i - 1).to.distance(legs.get(i).from) > 0.01) {
                    out.add(new Violation(Group.RULE, "GAP", id, "разрыв между участками " + i + " и " + (i + 1)));
                }
                double turn = turnDeg(legs.get(i - 1), legs.get(i));
                if (turn > Rules.MAX_TURN_DEG + 1e-6) {
                    out.add(new Violation(Group.RULE, "TURN", id, "поворот " + Math.round(turn) + "° между участками " + i + " и " + (i + 1)));
                }
            }
            List<Obstacle> own = ownPolygons(obstacles, br.target.xy);
            if (!own.isEmpty() && !legs.get(0).terminal) {
                out.add(new Violation(Group.RULE, "NO_TERMINAL_LEG", id, "нет финального прямого участка от границы здания до точки"));
            }
            Route.Leg last = legs.get(legs.size() - 1);
            if (last.to.distance(br.attach) > 0.01) {
                out.add(new Violation(Group.RULE, "NOT_TO_GOAL", id, "трасса не заканчивается в месте врезки"));
            }
            if (br.parent < 0) {
                Point end = F.createPoint(br.attach);
                if (br.existing.kind == Goals.Kind.NEW_CHAMBER && br.existing.segment.line.distance(end) > 0.05) {
                    out.add(new Violation(Group.RULE, "TIE_IN_OFF_NETWORK", id, "новая камера не лежит на существующем участке"));
                }
                if (br.existing.kind == Goals.Kind.EXISTING_CHAMBER && br.existing.chamber.xy.distance(br.attach) > 0.05) {
                    out.add(new Violation(Group.RULE, "TIE_IN_OFF_CHAMBER", id, "врезка не в существующей камере"));
                }
            } else {
                Branch parent = bs.get(br.parent);
                if (parent.line.distance(F.createPoint(br.attach)) > 0.05) {
                    out.add(new Violation(Group.RULE, "JOIN_OFF_BRANCH", id, "узел примыкания не лежит на ветви точки "
                            + parent.target.id.value()));
                }
                double pos = parent.project(br.attach);
                Route.Leg onto = parent.route.legs.get(parent.legAt(pos));
                if (onto.special || onto.terminal) {
                    out.add(new Violation(Group.RULE, "JOIN_ON_SPECIAL", id, "узел примыкания на специальном или заключительном участке"));
                }
                if (turnDeg(last, onto) > Rules.MAX_TURN_DEG + 1e-6) {
                    out.add(new Violation(Group.RULE, "JUNCTION_TURN", id, "на узле примыкания к ветви точки "
                            + parent.target.id.value() + " поворот " + Math.round(turnDeg(last, onto)) + "°"));
                }
            }
            // Участки труб на итоговом диаметре: отступы, специальные проходы, расход.
            double ownFlow = br.target.flow;
            for (TreeEvaluator.Piece p : pieces.get(b)) {
                Route.Leg src = legs.get(p.leg);
                Route.Leg synthetic = new Route.Leg(p.from, p.to, src.special, src.kSpec, src.crossed, src.terminal, src.tieInApproach);
                checkLeg(obstacles, p.dn, br.route.goal.exempt, synthetic, p.leg + 1, own, id, out);
                double pos = br.project(new Coordinate((p.from.x + p.to.x) / 2, (p.from.y + p.to.y) / 2));
                double expected = ownFlow;
                for (int c = 0; c < n; c++) {
                    if (bs.get(c).parent == b && bs.get(c).parentPos <= pos) {
                        expected += sub[c];
                    }
                }
                int idx = Rules.indexOfDn(p.dn);
                if (idx < 0 || Rules.CAPACITY[idx] + 1e-9 < expected) {
                    out.add(new Violation(Group.RULE, "CAPACITY", id, "участок " + (p.leg + 1) + ": ДУ " + p.dn + " не пропускает расход "
                            + Math.round(expected * 100) / 100.0 + " т/ч"));
                }
            }
        }
        // В узле новой сети проходящий ствол даёт два примыкания, значит ветвей в нём не более двух.
        for (int i = 0; i < n; i++) {
            int inNode = 1;
            for (int j = i + 1; j < n; j++) {
                if (bs.get(j).parent == bs.get(i).parent && bs.get(i).parent >= 0 && bs.get(i).attach.distance(bs.get(j).attach) < 0.05) {
                    inNode++;
                }
            }
            if (inNode > 2) {
                out.add(new Violation(Group.RULE, "CHAMBER_FULL", String.valueOf(bs.get(i).target.id.value()),
                        "в узле больше четырёх примыкающих участков"));
            }
        }
        // Диаметры не убывают к сети и не превышают предельную длину непрерывной части.
        for (int b = 0; b < n; b++) {
            List<TreeEvaluator.Piece> path = new ArrayList<>(pieces.get(b));
            int cur = b;
            while (bs.get(cur).parent >= 0) {
                Branch br = bs.get(cur);
                Branch parent = bs.get(br.parent);
                for (TreeEvaluator.Piece p : pieces.get(br.parent)) {
                    double pos = parent.project(new Coordinate((p.from.x + p.to.x) / 2, (p.from.y + p.to.y) / 2));
                    if (pos >= br.parentPos) {
                        path.add(p);
                    }
                }
                cur = br.parent;
            }
            String id = String.valueOf(bs.get(b).target.id.value());
            int run = -1;
            int i = 0;
            while (i < path.size()) {
                int dn = path.get(i).dn;
                if (dn < run) {
                    out.add(new Violation(Group.RULE, "DN_DECREASES", id, "диаметр уменьшается в сторону сети (ДУ " + dn + " после " + run + ")"));
                }
                run = Math.max(run, dn);
                int j = i;
                double len = 0;
                while (j < path.size() && path.get(j).dn == dn) {
                    len += path.get(j).length;
                    j++;
                }
                int idx = Rules.indexOfDn(dn);
                if (idx >= 0 && len > Rules.MAX_LENGTH[idx] + 1e-6) {
                    out.add(new Violation(Group.RULE, "MAX_LENGTH", id, "непрерывная часть ДУ " + dn + " длиной " + Math.round(len)
                            + " м больше предельной " + Math.round(Rules.MAX_LENGTH[idx]) + " м"));
                }
                i = j;
            }
        }
        // Структура: ветви пересекаются только в узле примыкания.
        for (int i = 0; i < n; i++) {
            for (int j = i + 1; j < n; j++) {
                Geometry common = bs.get(i).line.intersection(bs.get(j).line);
                if (common.isEmpty()) {
                    continue;
                }
                boolean joinPoint = bs.get(j).parent == i && common.getLength() < 0.05 && common.getNumPoints() >= 1
                        && allNear(common, bs.get(j).attach);
                // Две ветви, заканчивающиеся в одном узле общего ствола, касаются друг друга только в этой точке.
                boolean sameNode = bs.get(i).parent >= 0 && bs.get(i).parent == bs.get(j).parent && common.getLength() < 0.05
                        && bs.get(i).attach.distance(bs.get(j).attach) < 0.05 && allNear(common, bs.get(j).attach);
                joinPoint |= sameNode;
                if (!joinPoint) {
                    String a = String.valueOf(bs.get(i).target.id.value());
                    String c = String.valueOf(bs.get(j).target.id.value());
                    out.add(new Violation(Group.STRUCTURE, "ROUTES_OVERLAP", a, "ветви точек " + a + " и " + c
                            + (common.getLength() >= 0.05 ? " идут вместе " + Math.round(common.getLength()) + " м" : " пересекаются")
                            + " вне узла примыкания"));
                }
            }
        }
        return out;
    }

    private static boolean allNear(Geometry g, Coordinate c) {
        for (Coordinate x : g.getCoordinates()) {
            if (x.distance(c) > 0.05) {
                return false;
            }
        }
        return true;
    }
    private static double round2(double v) {
        return Math.round(v * 100) / 100.0;
    }
}