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
        /** Поиск отбрасывал пути из-за предельной длины: с большим диаметром (и большей длиной) маршрут мог бы найтись. */
        public boolean lengthLimited;
        /** Число меток, обработанных поиском (мера объёма работы). */
        public int labels;

        Result(Route route, String status) {
            this.route = route;
            this.status = status;
        }
    }

    private static final int STATIC_BUFFER_QUADRANTS = 4;
    /** Предел числа меток одного поиска: детерминированный предел работы (не зависит от загрузки машины). */
    /** Успешные поиски на данных ЗИЛ укладывались в тысячу меток; неудачные без предела перебирают всё пространство. */
    private static final int MAX_LABELS = 6_000;
    /**
     * Штраф за поворот в целевой функции поиска, м трубы: разница между двумя трассами равной длины решается в пользу
     * той, где меньше изломов (критерий приложения — количество поворотов, отсутствие мелкой ломаной). Реальная длина и
     * стоимость не меняются. Поворотом считается излом более чем на TURN_FREE_DEG градусов.
     */
    static final double TURN_PENALTY_M = 6;
    static final double TURN_FREE_DEG = 5;
    /** Для запасных подходов (здание, к которому строго по правилу не подойти) предел шире: пути там длиннее и таких целей единицы. */
    private static final int MAX_LABELS_FALLBACK = 40_000;
    /** Предел времени одного поиска, с: тяжёлые случаи (много специальных проходов) не должны задерживать весь расчёт. */
    static final double SEARCH_TIME_LIMIT_S = 20;
    /** Общий предел времени одного findRoute по всем коридорам, с. */
    static final double FIND_TIME_LIMIT_S = 40;
    private static final int MAX_GOAL_EDGES = 30;
    private static final double CROSSING_RAY_M = 600;
    private static final double CROSSING_RADIUS_M = 120;
    /** Не более стольких направлений на вершины и на цели для одной точки: остальные добавят соседние вершины. */
    private static final int MAX_VERTEX_DIRECTIONS = 6;
    private static final int MAX_GOAL_DIRECTIONS = 3;
    /** Коридор поиска: сумма расстояний от вершины до старта и до цели не больше k·прямое + запас. */
    private static final double[] CORRIDOR_FACTORS = {1.6, 3.0};
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
        return findRoute(start, goals, dn, profile, lengthBudget, null);
    }

    /** Поиск с преградой: трасса не должна пересекать barrier (уже построенные ветви), кроме как в самой точке присоединения. */
    public Result findRoute(Start start, List<Goal> goals, int dn, Profile profile, double lengthBudget, Geometry barrier) {
        if (goals.isEmpty()) {
            return new Result(null, "NO_GOALS");
        }
        double direct = Double.MAX_VALUE;
        for (Goal g : goals) {
            direct = Math.min(direct, start.origin.distance(g.xy));
        }
        String status = "NO_ROUTE_ON_GRAPH";
        boolean limited = false;
        long began = System.nanoTime();
        Route found = null;
        int totalLabels = 0;
        for (double factor : CORRIDOR_FACTORS) {
            if ((System.nanoTime() - began) / 1e9 > FIND_TIME_LIMIT_S) {
                break;
            }
            double limit = factor * direct + CORRIDOR_SLACK_M;
            Search search = new Search(start, goals, dn, profile, lengthBudget, limit, barrier);
            Route route = search.run();
            status = search.status;
            limited |= search.budgetPruned;
            totalLabels += search.labels;
            // Найденный путь короче границы коридора: он оптимален и по вершинам вне коридора.
            if (route != null && route.length <= limit) {
                Result ok = new Result(route, "OK");
                ok.labels = totalLabels;
                return ok;
            }
            if (route != null && (found == null || route.cost < found.cost)) {
                found = route;
            }
        }
        if (found != null) {
            // Путь длиннее границы последнего коридора: он допустим, хотя обход вне коридора мог быть короче.
            Result ok = new Result(found, "OK");
            ok.labels = totalLabels;
            return ok;
        }
        Result failed = new Result(null, status);
        failed.lengthLimited = limited;
        failed.labels = totalLabels;
        return failed;
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
            double axis = o.axisDistance(obstacles.zoneDn(dn));
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

        /** Геометрия специального прохода из точки по направлению (не зависит от пути, приведшего в точку). */
    private static final class Crossing {
        final Coordinate p1;
        final Coordinate p2;
        final double first;
        final double end;
        final double[] dir;
        final List<Obstacle> members;
        final double kMax;

        Crossing(Coordinate p1, Coordinate p2, double first, double end, double[] dir, List<Obstacle> members, double kMax) {
            this.p1 = p1;
            this.p2 = p2;
            this.first = first;
            this.end = end;
            this.dir = dir;
            this.members = members;
            this.kMax = kMax;
        }
    }

    private static final Object NO_CROSSING = new Object();
    /** Готовые специальные проходы: точка, ДУ и направление однозначно задают результат, поэтому он общий для всех поисков. */
    private final Map<String, Object> crossingMemo = new ConcurrentHashMap<>();

    /**
     * Видимость между вершинами для ДУ dn: для каждой вершины список видимых вершин, считается один раз при первом
     * обращении и переиспользуется всеми целями. Обычные участки не пользуются исключениями (отступ до собственного
     * здания снимается только на финальном участке), поэтому список от цели не зависит.
     */
    private final class Visibility {
        final int dn;
        final Coordinate[] v;
        final java.util.concurrent.atomic.AtomicReferenceArray<int[]> adj;

        Visibility(int dn) {
            this.dn = dn;
            this.v = vertices(dn);
            this.adj = new java.util.concurrent.atomic.AtomicReferenceArray<>(v.length);
        }

        int[] neighbors(int i) {
            int[] a = adj.get(i);
            if (a == null) {
                int[] tmp = new int[v.length];
                int n = 0;
                for (int j = 0; j < v.length; j++) {
                    if (j != i && obstacles.segmentClear(v[i], v[j], dn, null)) {
                        tmp[n++] = j;
                    }
                }
                a = Arrays.copyOf(tmp, n);
                adj.set(i, a);
            }
            return a;
        }
    }

    private Visibility visibility(int dn) {
        return visibilityCache.computeIfAbsent(obstacles.zoneDn(dn), Visibility::new);
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
        boolean budgetPruned;

        final Geometry barrierGeometry;
        final org.locationtech.jts.geom.prep.PreparedGeometry barrier;

        Search(Start start, List<Goal> goals, int dn, Profile profile, double budget, double limit, Geometry barrierGeometry) {
            this.barrierGeometry = barrierGeometry;
            this.barrier = barrierGeometry == null ? null : org.locationtech.jts.geom.prep.PreparedGeometryFactory.prepare(barrierGeometry);
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

        /** Отрезок ab пересекает преграду; касание в allowedEnd (место присоединения) допустимо. */
        boolean blocked(Coordinate a, Coordinate b, Coordinate allowedEnd) {
            if (barrier == null) {
                return false;
            }
            LineString seg = factory.createLineString(new Coordinate[]{a, b});
            if (!barrier.intersects(seg)) {
                return false;
            }
            if (allowedEnd == null) {
                return true;
            }
            Geometry inter = seg.intersection(barrierGeometry);
            if (inter.getLength() > 0.05) {
                return true;
            }
            for (Coordinate c : inter.getCoordinates()) {
                if (c.distance(allowedEnd) > 0.05) {
                    return true;
                }
            }
            return false;
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
            long deadline = System.nanoTime() + (long) (SEARCH_TIME_LIMIT_S * 1e9);
            while (!open.isEmpty()) {
                if (System.nanoTime() > deadline) {
                    status = "TIME_LIMIT";
                    return null;
                }
                Label l = open.poll();
                if (l.dead) {
                    continue;
                }
                if (l.v >= goalBase && l.v < goalBase + goals.size()) {
                    status = "OK";
                    return build(l, goals.get(l.v - goalBase));
                }
                if (++labels > (start.nearestBoundary ? MAX_LABELS : MAX_LABELS_FALLBACK)) {
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

        /** Косинус наибольшего поворота: сравнение скалярного произведения дешевле, чем acos на каждом ребре. */
        final double cosMaxTurn = Math.cos(Math.toRadians(maxTurnDeg + 1e-9));
        final double cosFreeTurn = Math.cos(Math.toRadians(TURN_FREE_DEG));

        boolean turnOk(double[] in, double[] out) {
            return in == null || in[0] * out[0] + in[1] * out[1] >= cosMaxTurn;
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
            if (d < 1e-6 || idx == l.v) {
                return;
            }
            if (l.length + d > budget) {
                budgetPruned = true;
                return;
            }
            double[] dir = Angles.unit(p, w);
            if (!turnOk(l.dir, dir) || (check && !obstacles.segmentClear(p, w, dn, null)) || blocked(p, w, null)) {
                return;
            }
            double cost = l.cost + d * price;
            double len = l.length + d;
            // Целевая функция линейна по длине и стоимости, поэтому приращение считается отдельно и к нему добавляется штраф.
            double turnCost = l.dir != null && l.dir[0] * dir[0] + l.dir[1] * dir[1] < cosFreeTurn ? TURN_PENALTY_M * perMeter() : 0;
            double obj = l.obj + weight(profile, d, d * price) + turnCost;
            push(new Label(idx, w, dir, len, cost, obj, l,
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
                        reach = Math.max(reach, o.axisDistance(obstacles.zoneDn(dn)));
                    }
                    double[] normal = {-g.lineDirection[1], g.lineDirection[0]};
                    for (int side : new int[]{1, -1}) {
                        double off = reach + 0.05;
                        Coordinate q = new Coordinate(g.xy.x + side * normal[0] * off, g.xy.y + side * normal[1] * off);
                        double d = p.distance(q);
                        double[] dir = Angles.unit(p, q);
                        if (d < 1e-6 || l.length + d > budget || !turnOk(l.dir, dir)
                                || obstacles.pointBlocked(q, dn, null) || !obstacles.segmentClear(p, q, dn, null) || blocked(p, q, null)) {
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
            if (d < 1e-6) {
                return;
            }
            if (l.length + extra + d > budget) {
                budgetPruned = true;
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
            if (g.joinDirection != null && Angles.turnDeg(dir, g.joinDirection) > maxTurnDeg + 1e-9) {
                return;
            }
            if (!obstacles.segmentClear(from, g.xy, dn, g.exempt) || blocked(from, g.xy, g.xy)) {
                return;
            }
            double cost = l.cost + (extra + d) * price;
            double len = l.length + extra + d;
            List<Route.Leg> legs = new ArrayList<>(before);
            legs.add(new Route.Leg(from, g.xy, false, 1, List.of(), false, true));
            push(new Label(goalBase + gi, g.xy, dir, len, cost, weight(profile, len, cost), l, legs), 0);
        }

        // -- специальные проходы через дороги и трамвайные пути

        /** Специальные области и возможные проходы вокруг точки: считаются один раз для каждой вершины графа. */
        final class CrossingInfo {
            final List<Obstacle> areas;
            final List<Crossing> crossings = new ArrayList<>();

            CrossingInfo(List<Obstacle> areas) {
                this.areas = areas;
            }
        }

        final Map<Integer, CrossingInfo> crossingCache = new HashMap<>();
        final Map<Long, Integer> dynamicIds = new HashMap<>();

        void addCrossings(Label l, Coordinate p) {
            CrossingInfo info = l.v < statics.length ? crossingCache.get(l.v) : null;
            if (info == null) {
                info = computeCrossings(p);
                if (l.v < statics.length) {
                    crossingCache.put(l.v, info);
                }
            }
            for (Crossing c : info.crossings) {
                if (turnOk(l.dir, c.dir)) {
                    if (l.length + c.end <= budget) {
                        pushCrossing(l, p, c);
                    } else {
                        budgetPruned = true;
                    }
                }
            }
        }

        CrossingInfo computeCrossings(Coordinate p) {
            Envelope env = new Envelope(p);
            env.expandBy(CROSSING_RADIUS_M);
            List<Obstacle> areas = new ArrayList<>();
            for (Obstacle o : obstacles.near(env, dn)) {
                if (o.crossable() && o.rule.kind == Rules.Kind.SPECIAL_AREA && o.geometry.getDimension() == 2) {
                    areas.add(o);
                }
            }
            CrossingInfo info = new CrossingInfo(areas);
            if (areas.isEmpty()) {
                return info;
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
            List<Coordinate> targets = new ArrayList<>();
            for (int idx : corridor) {
                Coordinate w = statics[idx];
                if (p.distance(w) < CROSSING_RADIUS_M) {
                    targets.add(w);
                }
            }
            targets.sort(Comparator.comparingDouble(w -> p.distance(w)));
            int added = 0;
            for (Coordinate w : targets) {
                if (added < MAX_VERTEX_DIRECTIONS && crossesAny(areas, p, w)) {
                    addDirection(directions, tried, Angles.unit(p, w));
                    added++;
                }
            }
            List<Goal> nearGoals = new ArrayList<>(goals);
            nearGoals.sort(Comparator.comparingDouble(g -> p.distance(g.xy)));
            added = 0;
            for (Goal g : nearGoals) {
                if (added < MAX_GOAL_DIRECTIONS && crossesAny(areas, p, g.xy)) {
                    addDirection(directions, tried, Angles.unit(p, g.xy));
                    added++;
                }
            }
            for (double[] u : directions) {
                String key = dn + ":" + Math.round(p.x * 1e3) + ":" + Math.round(p.y * 1e3) + ":"
                        + Math.round(Math.toDegrees(Math.atan2(u[1], u[0])) * 10);
                Object cached = crossingMemo.get(key);
                if (cached == null) {
                    Crossing computed = computeCrossing(p, u, areas);
                    cached = computed == null ? NO_CROSSING : computed;
                    crossingMemo.put(key, cached);
                }
                if (cached != NO_CROSSING) {
                    info.crossings.add((Crossing) cached);
                }
            }
            return info;
        }

        boolean crossesAny(List<Obstacle> areas, Coordinate a, Coordinate b) {
            Envelope segEnv = new Envelope(a, b);
            LineString seg = null;
            for (Obstacle o : areas) {
                if (!o.geometry.getEnvelopeInternal().intersects(segEnv)) {
                    continue;
                }
                if (seg == null) {
                    seg = factory.createLineString(new Coordinate[]{a, b});
                }
                if (obstacles.prepared(o).intersects(seg)) {
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

        /** Одна попытка: прямой специальный проход из p по направлению u; null, если он невозможен. */
        Crossing computeCrossing(Coordinate p, double[] u, List<Obstacle> areas) {
            Coordinate far = new Coordinate(p.x + u[0] * CROSSING_RAY_M, p.y + u[1] * CROSSING_RAY_M);
            LineString ray = factory.createLineString(new Coordinate[]{p, far});
            Envelope rayEnv = ray.getEnvelopeInternal();
            // Интервалы луча внутри полигонов: [вход, выход] вдоль луча и объект.
            List<double[]> intervals = new ArrayList<>();
            List<Obstacle> owners = new ArrayList<>();
            for (Obstacle o : areas) {
                if (!o.geometry.getEnvelopeInternal().intersects(rayEnv) || !obstacles.prepared(o).intersects(ray)) {
                    continue;
                }
                Geometry inside;
                try {
                    inside = ray.intersection(o.geometry);
                } catch (RuntimeException e) {
                    continue;
                }
                for (int i = 0; i < inside.getNumGeometries(); i++) {
                    Geometry piece = inside.getGeometryN(i);
                    if (piece.getDimension() != 1 || piece.isEmpty()) {
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
                return null;
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
                return null; // специальная полоса начиналась бы позади точки: поворот внутри прохода недопустим
            }
            // Угол входа не меньше заданного для каждого пересекаемого объекта.
            for (int i = 0; i < members.size(); i++) {
                Obstacle m = members.get(i);
                if (m.rule.minAngleDeg > 0) {
                    Coordinate e = new Coordinate(p.x + u[0] * entries.get(i), p.y + u[1] * entries.get(i));
                    double[] tangent = boundaryTangent(m.geometry, e);
                    if (tangent == null || Angles.acuteDeg(u, tangent) < m.rule.minAngleDeg - 1e-9) {
                        return null;
                    }
                }
            }
            Coordinate p1 = new Coordinate(p.x + u[0] * first, p.y + u[1] * first);
            Coordinate p2 = new Coordinate(p.x + u[0] * end, p.y + u[1] * end);
            Set<Obstacle> crossed = new HashSet<>(members);
            if (first > 1e-6 && !obstacles.segmentClear(p, p1, dn, null)) {
                return null;
            }
            if (!obstacles.segmentClear(p1, p2, dn, crossed)) {
                return null;
            }
            // Вне специального интервала отступ до пересекаемых объектов обязан соблюдаться: его концы за пределами зон.
            for (Obstacle m : members) {
                ObstacleSet.Zone z = obstacles.zone(m, dn);
                if (z.geometry.intersects(factory.createPoint(p1)) || z.geometry.intersects(factory.createPoint(p2))) {
                    return null;
                }
            }
            return new Crossing(p1, p2, first, end, u, members, kMax);
        }

        void pushCrossing(Label l, Coordinate p, Crossing c) {
            if (blocked(p, c.p2, null)) {
                return;
            }
            List<Route.Leg> legs = new ArrayList<>();
            double addLength = 0;
            double addCost = 0;
            if (c.first > 1e-6) {
                legs.add(new Route.Leg(p, c.p1, false, 1, List.of(), false, false));
                addLength += c.first;
                addCost += c.first * price;
            }
            double special = c.end - c.first;
            legs.add(new Route.Leg(c.p1, c.p2, true, c.kMax, new ArrayList<>(c.members), false, false));
            addLength += special;
            addCost += special * price * c.kMax;
            double len = l.length + addLength;
            double cost = l.cost + addCost;
            // Конец прохода получает устойчивый номер по координате: иначе метки не сравнивались бы между собой и множились.
            long key = Math.round(c.p2.x * 100) * 1_000_003L + Math.round(c.p2.y * 100);
            int id = dynamicIds.computeIfAbsent(key, k -> nextDynamic++);
            push(new Label(id, c.p2, c.dir, len, cost, weight(profile, len, cost), l, legs), nearestGoalDistance(c.p2));
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
