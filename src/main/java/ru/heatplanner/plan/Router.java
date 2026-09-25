package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineSegment;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.algorithm.Orientation;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.operation.buffer.BufferOp;
import org.locationtech.jts.operation.buffer.BufferParameters;
import org.locationtech.jts.operation.distance.DistanceOp;
import org.locationtech.jts.simplify.DouglasPeuckerSimplifier;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Поиск одной допустимой трассы: расширенный граф видимости и A* с состоянием направления и метками длины и цены.
 *
 * Граф. Вершины — точки вокруг запретных зон на расстоянии чуть больше требуемого отступа для ДУ: только из них
 * имеет смысл делать поворот при обходе препятствия. К ним добавляются старт, места врезки и концы специальных
 * проходов. Рёбра проверяются лениво по настоящей геометрии всей длины отрезка и кэшируются в пределах поиска.
 *
 * Специальный проход через полигон дороги или трамвая — атомарный переход: один прямой участок от точки за 3 м до входа
 * до точки за 3 м после выхода, угол входа не меньше 45°; посередине его нельзя ни закончить, ни повернуть.
 *
 * A*. Состояние — вершина и направление прихода, потому что поворот не может превышать 90°. Метки хранят пройденную
 * длину и цену. Метка отбрасывается только если в том же состоянии есть метка не хуже по обоим показателям: более
 * дешёвый, но длинный путь может не пройти предельную длину ДУ. Эвристика — прямое расстояние до ближайшей врезки,
 * умноженное на цену метра при K ≥ 1, то есть нижняя оценка.
 *
 * Поиск эвристический: он ищет на построенном графе, поэтому «путь не найден» означает только
 * «алгоритм не нашёл допустимый маршрут», а не доказательство отсутствия подключения.
 */
public final class Router {

    /** Чем измеряется качество трассы при поиске; итоговая стоимость всегда считается по приложению. */
    public enum Profile {
        /** 0,7·цена/25 млн + 0,3·длина/100 — показатель ранжирования. */
        BALANCED,
        ECONOMIC,
        COMPACT
    }

    /** Результат поиска: трасса или причина, по которой она не найдена. */
    public static final class Result {
        public final Route route;
        public final String status;

        Result(Route route, String status) {
            this.route = route;
            this.status = status;
        }
    }

    private static final int STATIC_BUFFER_QUADRANTS = 4;
    private static final int MAX_LABELS = 300_000;
    private static final int MAX_GOAL_EDGES = 30;
    private static final double CROSSING_RAY_M = 600;
    private static final double CROSSING_RADIUS_M = 400;
    /** Коридор поиска: сумма расстояний от вершины до старта и до цели не больше k·прямое + запас. */
    private static final double[] CORRIDOR_FACTORS = {1.6, 3.0, Double.POSITIVE_INFINITY};
    private static final double CORRIDOR_SLACK_M = 120;
    /**
     * Допуск упрощения колец вершин, м. Хорда между оставшимися вершинами уходит внутрь кольца не более чем на этот допуск,
     * поэтому кольцо вершин строится дальше от ограничения ровно на эту величину и хорда не срезает запретную зону.
     */
    private static final double RING_SIMPLIFY_M = 0.3;

    private final ObstacleSet obstacles;
    private final RuleSet rules;
    private final GeometryFactory factory = new GeometryFactory();
    private final Map<Integer, Visibility> visibilityCache = new ConcurrentHashMap<>();
    /** Наибольший поворот, градусы. По приложению 90°; изменяется только в исследовательских проверках. */
    double maxTurnDeg = Rules.MAX_TURN_DEG;

    public Router(ObstacleSet obstacles, RuleSet rules) {
        this.obstacles = obstacles;
        this.rules = rules;
    }

    /** Лучшая по цели профиля трасса среди нескольких стартов (подходов к точке подключения). */
    public Result findBest(List<Start> starts, List<Goal> goals, int dn, Profile profile, double lengthBudget) {
        Route best = null;
        String status = "NO_START: к точке подключения нельзя подойти допустимым финальным участком";
        for (Start s : starts) {
            Result r = findRoute(s, goals, dn, profile, lengthBudget);
            if (r.route != null && (best == null || weight(profile, r.route.length, r.route.cost)
                    < weight(profile, best.length, best.cost))) {
                best = r.route;
            }
            if (best == null) {
                status = r.status;
            }
        }
        return new Result(best, best != null ? "OK" : status);
    }

    public Result findRoute(Start start, List<Goal> goals, int dn, Profile profile, double lengthBudget) {
        if (goals.isEmpty()) {
            return new Result(null, "NO_GOALS");
        }
        double direct = Double.MAX_VALUE;
        for (Goal g : goals) {
            direct = Math.min(direct, start.origin.distance(g.xy));
        }
        String status = "NO_ROUTE_ON_GRAPH";
        for (double factor : CORRIDOR_FACTORS) {
            double limit = factor * direct + CORRIDOR_SLACK_M;
            Search search = new Search(start, goals, dn, profile, lengthBudget, limit);
            Route route = search.run();
            status = search.status;
            // Найденный путь короче границы коридора: он оптимален и по вершинам вне коридора.
            if (route != null && (route.length <= limit || Double.isInfinite(factor))) {
                return new Result(route, "OK");
            }
        }
        return new Result(null, status);
    }

    // ---- Вес и эвристика ---------------------------------------------------------------------------------------------

    private static double weight(Profile p, double length, double cost) {
        switch (p) {
            case ECONOMIC:
                return cost;
            case COMPACT:
                return length;
            default:
                return 0.7 * cost / 25_000_000 + 0.3 * length / 100;
        }
    }

    // ---- Вершины графа --------------------------------------------------------------------------------------------------

    /**
     * Вершины вокруг запретных зон для ДУ dn. Дуги буфера при вершинах на окружности радиуса d срезаются хордами не
     * глубже d·(1 − cos(α/2)), поэтому радиус увеличен так, чтобы хорды оставались за границей точной зоны.
     * Кольца упрощаются только на прямых участках (см. RING_SIMPLIFY_M).
     */
    private Coordinate[] vertices(int dn) {
        double alpha = Math.PI / (2 * STATIC_BUFFER_QUADRANTS);
        double cosHalf = Math.cos(alpha / 2);
        List<Coordinate> out = new ArrayList<>();
        Set<Long> seen = new HashSet<>();
        BufferParameters params = new BufferParameters(STATIC_BUFFER_QUADRANTS, BufferParameters.CAP_ROUND,
                BufferParameters.JOIN_ROUND, 5.0);
        for (Obstacle o : obstacles.all()) {
            // Запас над зоной покрывает срез дуги хордами и упрощение кольца, поэтому рёбра вдоль колец остаются допустимыми.
            double axis = o.axisDistance(dn);
            double d = axis + rules.geometryEps + axis * (1 - cosHalf) + RING_SIMPLIFY_M + 0.05;
            Geometry buffer = DouglasPeuckerSimplifier.simplify(BufferOp.bufferOp(o.geometry, d, params), RING_SIMPLIFY_M);
            for (int i = 0; i < buffer.getNumGeometries(); i++) {
                if (!(buffer.getGeometryN(i) instanceof Polygon)) {
                    continue;
                }
                Polygon poly = (Polygon) buffer.getGeometryN(i);
                // Внешнее кольцо: поворачивать вокруг здания имеет смысл только в выпуклых вершинах.
                addRing(out, seen, poly.getExteriorRing().getCoordinates(), dn, true);
                for (int h = 0; h < poly.getNumInteriorRing(); h++) {
                    addRing(out, seen, poly.getInteriorRingN(h).getCoordinates(), dn, false);
                }
            }
        }
        return out.toArray(new Coordinate[0]);
    }

    private void addRing(List<Coordinate> out, Set<Long> seen, Coordinate[] ring, int dn, boolean convexOnly) {
        boolean ccw = Orientation.isCCW(ring);
        int n = ring.length - 1;
        for (int i = 0; i < n; i++) {
            Coordinate c = ring[i];
            if (convexOnly) {
                Coordinate prev = ring[(i + n - 1) % n];
                Coordinate next = ring[(i + 1) % n];
                double cross = (c.x - prev.x) * (next.y - c.y) - (c.y - prev.y) * (next.x - c.x);
                // Вогнутая вершина кольца (поворот против направления обхода) не бывает точкой поворота кратчайшего пути.
                if (ccw ? cross < 0 : cross > 0) {
                    continue;
                }
            }
            long key = Math.round(c.x / 0.05) * 1_000_003L + Math.round(c.y / 0.05);
            if (seen.add(key) && !obstacles.pointBlocked(c, dn, null)) {
                out.add(new Coordinate(c.x, c.y));
            }
        }
    }

    /**
     * Видимость между вершинами для ДУ dn: для каждой вершины список видимых вершин, считается один раз при первом
     * обращении и переиспользуется всеми целями. Обычные участки не пользуются исключениями (отступ до собственного
     * здания снимается только на финальном участке), поэтому список от цели не зависит.
     */
    private final class Visibility {
        final int dn;
        final Coordinate[] v;
        final int[][] adj;

        Visibility(int dn) {
            this.dn = dn;
            this.v = vertices(dn);
            this.adj = new int[v.length][];
        }

        int[] neighbors(int i) {
            int[] a = adj[i];
            if (a == null) {
                int[] tmp = new int[v.length];
                int n = 0;
                for (int j = 0; j < v.length; j++) {
                    if (j != i && obstacles.segmentClear(v[i], v[j], dn, null)) {
                        tmp[n++] = j;
                    }
                }
                a = Arrays.copyOf(tmp, n);
                adj[i] = a;
            }
            return a;
        }
    }

    private Visibility visibility(int dn) {
        return visibilityCache.computeIfAbsent(dn, Visibility::new);
    }

    // ---- Один поиск --------------------------------------------------------------------------------------------------

    private static final class Label {
        final int v;
        final int sector;
        final Coordinate at;
        final double[] dir;
        final double length;
        final double cost;
        final double obj;
        /** Приоритет в очереди: вес пути плюс оценка остатка; задаётся при постановке в очередь. */
        double f;
        final Label parent;
        final List<Route.Leg> legs;
        boolean dead;

        Label(int v, Coordinate at, double[] dir, double length, double cost, double obj, Label parent,
              List<Route.Leg> legs) {
            this.v = v;
            this.at = at;
            this.dir = dir;
            this.sector = dir == null ? -1 : (int) Math.floor((Math.atan2(dir[1], dir[0]) + Math.PI) / (Math.PI / 12));
            this.length = length;
            this.cost = cost;
            this.obj = obj;
            this.parent = parent;
            this.legs = legs;
        }
    }

    private final class Search {
        final Start start;
        final List<Goal> goals;
        final int dn;
        final Profile profile;
        final double budget;
        final double limit;
        final double price;
        final Visibility vis;
        final Coordinate[] statics;
        final boolean[] inCorridor;
        final int[] corridor;
        final int startVertex;
        final int goalBase;
        int nextDynamic;
        final Map<Long, List<Label>> states = new HashMap<>();
        final PriorityQueue<Label> open = new PriorityQueue<>(Comparator.comparingDouble((Label l) -> l.f));
        String status = "NO_ROUTE_ON_GRAPH";
        int labels;

        Search(Start start, List<Goal> goals, int dn, Profile profile, double budget, double limit) {
            this.start = start;
            this.goals = goals;
            this.dn = dn;
            this.profile = profile;
            this.budget = budget;
            this.limit = limit;
            this.price = Rules.PRICE[Rules.indexOfDn(dn)];
            this.vis = visibility(dn);
            this.statics = vis.v;
            this.startVertex = statics.length;
            this.goalBase = statics.length + 1;
            this.nextDynamic = goalBase + goals.size();
            this.inCorridor = new boolean[statics.length];
            List<Integer> in = new ArrayList<>();
            for (int i = 0; i < statics.length; i++) {
                if (start.origin.distance(statics[i]) + nearestGoalDistance(statics[i]) <= limit) {
                    inCorridor[i] = true;
                    in.add(i);
                }
            }
            this.corridor = in.stream().mapToInt(Integer::intValue).toArray();
        }

        double nearestGoalDistance(Coordinate c) {
            double best = Double.MAX_VALUE;
            for (Goal g : goals) {
                best = Math.min(best, c.distance(g.xy));
            }
            return best;
        }

        double perMeter() {
            switch (profile) {
                case ECONOMIC:
                    return price;
                case COMPACT:
                    return 1;
                default:
                    return 0.7 * price / 25_000_000 + 0.3 / 100;
            }
        }

        Route run() {
            double startLength = start.terminalPoint == null ? 0 : start.origin.distance(start.terminalPoint);
            double startCost = startLength * price;
            Label first = new Label(startVertex, start.origin, start.direction, startLength, startCost,
                    weight(profile, startLength, startCost), null, Collections.emptyList());
            push(first, nearestGoalDistance(start.origin));
            while (!open.isEmpty()) {
                Label l = open.poll();
                if (l.dead) {
                    continue;
                }
                if (l.v >= goalBase && l.v < goalBase + goals.size()) {
                    status = "OK";
                    return build(l, goals.get(l.v - goalBase));
                }
                if (++labels > MAX_LABELS) {
                    status = "SEARCH_BUDGET_EXHAUSTED";
                    return null;
                }
                expand(l);
            }
            return null;
        }

        /** Ставит метку в очередь, если её не превосходит метка того же состояния (вершина и сектор направления). */
        void push(Label l, double h) {
            long key = ((long) l.v << 32) | (l.sector + 1L);
            List<Label> list = states.computeIfAbsent(key, k -> new ArrayList<>(2));
            for (Label o : list) {
                if (!o.dead && o.length <= l.length + 1e-9 && o.obj <= l.obj + 1e-12) {
                    return;
                }
            }
            for (Label o : list) {
                if (!o.dead && l.length <= o.length + 1e-9 && l.obj <= o.obj + 1e-12) {
                    o.dead = true;
                }
            }
            list.add(l);
            l.f = l.obj + h * perMeter();
            open.add(l);
        }

        boolean turnOk(double[] in, double[] out) {
            return in == null || Angles.turnDeg(in, out) <= maxTurnDeg + 1e-9;
        }

        void expand(Label l) {
            Coordinate p = l.at;
            addGoalEdges(l, p);
            addStaticEdges(l, p);
            addCrossings(l, p);
        }

        // -- обычные переходы между вершинами

        void addStaticEdges(Label l, Coordinate p) {
            if (l.v < statics.length) {
                for (int idx : vis.neighbors(l.v)) {
                    if (inCorridor[idx]) {
                        step(l, p, idx, false);
                    }
                }
            } else {
                for (int idx : corridor) {
                    step(l, p, idx, true);
                }
            }
        }

        void step(Label l, Coordinate p, int idx, boolean check) {
            Coordinate w = statics[idx];
            double d = p.distance(w);
            if (d < 1e-6 || l.length + d > budget || idx == l.v) {
                return;
            }
            double[] dir = Angles.unit(p, w);
            if (!turnOk(l.dir, dir) || (check && !obstacles.segmentClear(p, w, dn, null))) {
                return;
            }
            double cost = l.cost + d * price;
            double len = l.length + d;
            push(new Label(idx, w, dir, len, cost, weight(profile, len, cost), l,
                    List.of(new Route.Leg(p, w, false, 1, List.of(), false, false))), nearestGoalDistance(w));
        }

        // -- заключительный подход к врезке

        void addGoalEdges(Label l, Coordinate p) {
            List<Integer> order = new ArrayList<>();
            for (int i = 0; i < goals.size(); i++) {
                order.add(i);
            }
            order.sort(Comparator.comparingDouble(i -> p.distance(goals.get(i).xy)));
            int n = 0;
            for (int gi : order) {
                if (n++ >= MAX_GOAL_EDGES) {
                    break;
                }
                Goal g = goals.get(gi);
                if (p.distance(g.xy) < 1e-6) {
                    continue;
                }
                finalApproach(l, p, gi, g, p, List.of());
                if (g.lineDirection != null) {
                    // Подход не вдоль существующей линии: сначала к точке у отступа по нормали, затем прямо к врезке.
                    double reach = 0;
                    for (Obstacle o : g.exempt) {
                        reach = Math.max(reach, o.axisDistance(dn));
                    }
                    double[] normal = {-g.lineDirection[1], g.lineDirection[0]};
                    for (int side : new int[]{1, -1}) {
                        double off = reach + 0.05;
                        Coordinate q = new Coordinate(g.xy.x + side * normal[0] * off, g.xy.y + side * normal[1] * off);
                        double d = p.distance(q);
                        double[] dir = Angles.unit(p, q);
                        if (d < 1e-6 || l.length + d > budget || !turnOk(l.dir, dir)
                                || obstacles.pointBlocked(q, dn, null) || !obstacles.segmentClear(p, q, dn, null)) {
                            continue;
                        }
                        finalApproach(l, q, gi, g, p, List.of(new Route.Leg(p, q, false, 1, List.of(), false, false)));
                    }
                }
            }
        }

        /**
         * Последний прямой участок от точки from к врезке g. before — участки, приведшие в from от метки l
         * (пусто, если from — сама точка метки). Отступ до существующей сети, к которой идёт врезка, на нём снят,
         * но угол подхода к линии не меньше заданного.
         */
        void finalApproach(Label l, Coordinate from, int gi, Goal g, Coordinate labelPoint, List<Route.Leg> before) {
            double d = from.distance(g.xy);
            double extra = before.isEmpty() ? 0 : labelPoint.distance(from);
            if (d < 1e-6 || l.length + extra + d > budget) {
                return;
            }
            double[] dir = Angles.unit(from, g.xy);
            double[] incoming = before.isEmpty() ? l.dir : Angles.unit(labelPoint, from);
            if (!turnOk(incoming, dir)) {
                return;
            }
            if (g.lineDirection != null && Angles.acuteDeg(dir, g.lineDirection) < rules.tieInApproachMinAngleDeg - 1e-9) {
                return;
            }
            if (!obstacles.segmentClear(from, g.xy, dn, g.exempt)) {
                return;
            }
            double cost = l.cost + (extra + d) * price;
            double len = l.length + extra + d;
            List<Route.Leg> legs = new ArrayList<>(before);
            legs.add(new Route.Leg(from, g.xy, false, 1, List.of(), false, true));
            push(new Label(goalBase + gi, g.xy, dir, len, cost, weight(profile, len, cost), l, legs), 0);
        }

        // -- специальные проходы через дороги и трамвайные пути

        void addCrossings(Label l, Coordinate p) {
            Envelope env = new Envelope(p);
            env.expandBy(CROSSING_RADIUS_M);
            List<Obstacle> areas = new ArrayList<>();
            for (Obstacle o : obstacles.near(env, dn)) {
                if (o.crossable() && o.rule.kind == Rules.Kind.SPECIAL_AREA && o.geometry.getDimension() == 2) {
                    areas.add(o);
                }
            }
            if (areas.isEmpty()) {
                return;
            }
            Set<Long> tried = new LinkedHashSet<>();
            List<double[]> directions = new ArrayList<>();
            org.locationtech.jts.geom.Point pp = factory.createPoint(p);
            for (Obstacle o : areas) {
                Coordinate[] near = DistanceOp.nearestPoints(o.geometry, pp);
                double[] u0 = Angles.unit(p, near[0]);
                for (int deg : new int[]{0, 15, -15, 30, -30}) {
                    double a = Math.atan2(u0[1], u0[0]) + Math.toRadians(deg);
                    addDirection(directions, tried, new double[]{Math.cos(a), Math.sin(a)});
                }
            }
            // Направления на вершины и цели, чей прямой путь упирается в специальную зону.
            for (int idx : corridor) {
                Coordinate w = statics[idx];
                if (p.distance(w) < CROSSING_RADIUS_M && crossesAny(areas, p, w)) {
                    addDirection(directions, tried, Angles.unit(p, w));
                }
            }
            for (Goal g : goals) {
                if (crossesAny(areas, p, g.xy)) {
                    addDirection(directions, tried, Angles.unit(p, g.xy));
                }
            }
            for (double[] u : directions) {
                if (turnOk(l.dir, u)) {
                    tryCrossing(l, p, u, areas);
                }
            }
        }

        boolean crossesAny(List<Obstacle> areas, Coordinate a, Coordinate b) {
            LineString seg = factory.createLineString(new Coordinate[]{a, b});
            for (Obstacle o : areas) {
                if (o.geometry.intersects(seg)) {
                    return true;
                }
            }
            return false;
        }

        void addDirection(List<double[]> out, Set<Long> tried, double[] u) {
            long key = Math.round(Math.toDegrees(Math.atan2(u[1], u[0])) * 2);
            if (tried.add(key)) {
                out.add(u);
            }
        }

        /** Одна попытка: прямой специальный проход из p по направлению u. */
        void tryCrossing(Label l, Coordinate p, double[] u, List<Obstacle> areas) {
            Coordinate far = new Coordinate(p.x + u[0] * CROSSING_RAY_M, p.y + u[1] * CROSSING_RAY_M);
            LineString ray = factory.createLineString(new Coordinate[]{p, far});
            // Интервалы луча внутри полигонов: [вход, выход] вдоль луча и объект.
            List<double[]> intervals = new ArrayList<>();
            List<Obstacle> owners = new ArrayList<>();
            for (Obstacle o : areas) {
                Geometry inside;
                try {
                    inside = ray.intersection(o.geometry);
                } catch (RuntimeException e) {
                    continue;
                }
                for (int i = 0; i < inside.getNumGeometries(); i++) {
                    Geometry piece = inside.getGeometryN(i);
                    if (piece.getDimension() != 1) {
                        continue;
                    }
                    Coordinate[] cs = piece.getCoordinates();
                    double t0 = p.distance(cs[0]);
                    double t1 = p.distance(cs[cs.length - 1]);
                    intervals.add(new double[]{Math.min(t0, t1), Math.max(t0, t1)});
                    owners.add(o);
                }
            }
            if (intervals.isEmpty()) {
                return;
            }
            Integer[] order = new Integer[intervals.size()];
            for (int i = 0; i < order.length; i++) {
                order[i] = i;
            }
            Arrays.sort(order, Comparator.comparingDouble(i -> intervals.get(i)[0]));
            // Первое пересечение и цепочка интервалов, чьи специальные полосы сливаются.
            List<Obstacle> members = new ArrayList<>();
            List<Double> entries = new ArrayList<>();
            double first = intervals.get(order[0])[0] - owners.get(order[0]).rule.extent;
            double end = intervals.get(order[0])[1] + owners.get(order[0]).rule.extent;
            members.add(owners.get(order[0]));
            entries.add(intervals.get(order[0])[0]);
            double kMax = owners.get(order[0]).rule.kSpec;
            for (int k = 1; k < order.length; k++) {
                Obstacle o = owners.get(order[k]);
                double[] iv = intervals.get(order[k]);
                if (iv[0] - o.rule.extent <= end + 1e-9) {
                    end = Math.max(end, iv[1] + o.rule.extent);
                    if (!members.contains(o)) {
                        members.add(o);
                        entries.add(iv[0]);
                    }
                    kMax = Math.max(kMax, o.rule.kSpec);
                } else {
                    break;
                }
            }
            if (first < -1e-9) {
                return; // специальная полоса начиналась бы позади точки: поворот внутри прохода недопустим
            }
            // Угол входа не меньше заданного для каждого пересекаемого объекта.
            for (int i = 0; i < members.size(); i++) {
                Obstacle m = members.get(i);
                if (m.rule.minAngleDeg > 0) {
                    Coordinate e = new Coordinate(p.x + u[0] * entries.get(i), p.y + u[1] * entries.get(i));
                    double[] tangent = boundaryTangent(m.geometry, e);
                    if (tangent == null || Angles.acuteDeg(u, tangent) < m.rule.minAngleDeg - 1e-9) {
                        return;
                    }
                }
            }
            if (l.length + end > budget) {
                return;
            }
            Coordinate p1 = new Coordinate(p.x + u[0] * first, p.y + u[1] * first);
            Coordinate p2 = new Coordinate(p.x + u[0] * end, p.y + u[1] * end);
            Set<Obstacle> crossed = new HashSet<>(members);
            if (first > 1e-6 && !obstacles.segmentClear(p, p1, dn, null)) {
                return;
            }
            if (!obstacles.segmentClear(p1, p2, dn, crossed)) {
                return;
            }
            // Вне специального интервала отступ до пересекаемых объектов обязан соблюдаться: его концы за пределами зон.
            for (Obstacle m : members) {
                ObstacleSet.Zone z = obstacles.zone(m, dn);
                if (z.geometry.intersects(factory.createPoint(p1)) || z.geometry.intersects(factory.createPoint(p2))) {
                    return;
                }
            }
            List<Route.Leg> legs = new ArrayList<>();
            double addLength = 0;
            double addCost = 0;
            if (first > 1e-6) {
                legs.add(new Route.Leg(p, p1, false, 1, List.of(), false, false));
                addLength += first;
                addCost += first * price;
            }
            double special = end - first;
            legs.add(new Route.Leg(p1, p2, true, kMax, new ArrayList<>(members), false, false));
            addLength += special;
            addCost += special * price * kMax;
            double len = l.length + addLength;
            double cost = l.cost + addCost;
            push(new Label(nextDynamic++, p2, u, len, cost, weight(profile, len, cost), l, legs), nearestGoalDistance(p2));
        }

        Route build(Label last, Goal goal) {
            List<Route.Leg> reversed = new ArrayList<>();
            for (Label l = last; l != null; l = l.parent) {
                for (int i = l.legs.size() - 1; i >= 0; i--) {
                    reversed.add(l.legs.get(i));
                }
            }
            Collections.reverse(reversed);
            List<Route.Leg> legs = new ArrayList<>();
            if (start.terminalPoint != null) {
                legs.add(new Route.Leg(start.terminalPoint, start.origin, false, 1, List.of(), true, false));
            }
            legs.addAll(reversed);
            return new Route(legs, start, goal, dn);
        }
    }
    /** Направление границы полигона у точки e: ближайший отрезок границы (единичный вектор) или null. */
    private static double[] boundaryTangent(Geometry area, Coordinate e) {
        double best = Double.MAX_VALUE;
        double[] tangent = null;
        for (int i = 0; i < area.getNumGeometries(); i++) {
            Polygon poly = (Polygon) area.getGeometryN(i);
            List<Coordinate[]> rings = new ArrayList<>();
            rings.add(poly.getExteriorRing().getCoordinates());
            for (int h = 0; h < poly.getNumInteriorRing(); h++) {
                rings.add(poly.getInteriorRingN(h).getCoordinates());
            }
            for (Coordinate[] ring : rings) {
                for (int k = 1; k < ring.length; k++) {
                    double d = new LineSegment(ring[k - 1], ring[k]).distance(e);
                    if (d < best) {
                        best = d;
                        tangent = Angles.unit(ring[k - 1], ring[k]);
                    }
                }
            }
        }
        return tangent;
    }
}
