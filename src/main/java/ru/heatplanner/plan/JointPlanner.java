package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.linearref.LengthIndexedLine;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Совместное подключение: цели добавляются в сеть по одной, и каждая либо идёт к существующей сети, либо примыкает
 * к уже построенной ветви (в точке примыкания появляется новая камера). Из вариантов выбирается тот, при котором
 * вся сеть после добавления выходит лучше по показателю ранжирования; стоимость, диаметры и допустимость считает
 * {@link TreeEvaluator} на всём дереве, а не на одной трассе.
 */
public final class JointPlanner {

    /** Итог по цели, которую подключить не удалось. */
    public static final class Unconnected {
        public final PlanInput.Target target;
        public final String reason;
        /** Подойти к точке от здания не удалось вовсе (причина из правила подхода), а не «не нашли маршрут». */
        public final boolean startRefused;

        Unconnected(PlanInput.Target target, String reason, boolean startRefused) {
            this.target = target;
            this.reason = reason;
            this.startRefused = startRefused;
        }
    }

    public static final class Solution {
        public final List<Branch> branches;
        public final List<Unconnected> unconnected;
        public final TreeEvaluator.Result evaluation;
        public final double penalty;

        Solution(List<Branch> branches, List<Unconnected> unconnected, TreeEvaluator.Result evaluation) {
            this.branches = branches;
            this.unconnected = unconnected;
            this.evaluation = evaluation;
            double p = 0;
            for (Unconnected u : unconnected) {
                p += Rules.penalty(u.target.flow);
            }
            this.penalty = p;
        }

        public double calculatedCost() {
            return evaluation.cost() + penalty;
        }

        public double score() {
            return Rules.score(calculatedCost(), evaluation.length);
        }
    }

    private static final GeometryFactory F = new GeometryFactory();
    /** К стольким ближайшим ветвям строится отдельная трасса-кандидат. */
    private static final int NEAREST_BRANCHES = 3;
    private static final int MAX_BRANCH_GOALS = 40;
    /** Диаметр повышается из-за предельной длины не более чем на столько шагов: дальше трасса всё равно не осмысленна. */
    private static final int MAX_DN_STEPS = 4;
    private static final double BRANCH_GOAL_STEP_M = 30;

    /** Отладочный вывод кандидатов (только для исследований). */
    static boolean debug = false;

    /** Запас (м) к отступам при поиске: диаметр ветви потом может вырасти, а отступ растёт вместе с шириной пары труб. */
    private static final double ROUTING_MARGIN_M = 0.75;
    private static final ExecutorService POOL = Executors.newFixedThreadPool(Math.max(2, Runtime.getRuntime().availableProcessors() - 1), r -> {
        Thread t = new Thread(r, "joint-planner");
        t.setDaemon(true);
        return t;
    });

    private final PlanInput in;
    /** Ограничения с запасом: по ним ищутся трассы. */
    private final ObstacleSet obstacles;
    /** Ограничения без запаса: по ним проверяется итоговый диаметр. */
    private final ObstacleSet exact;
    private final Router router;
    private volatile Router exactRouter;
    private final RuleSet exactRules;
    private final RuleSet rules;
    private volatile Router.Profile profile = Router.Profile.BALANCED;

    public JointPlanner(PlanInput in, ObstacleSet exact, RuleSet rules) {
        this.in = in;
        this.exact = exact;
        this.exactRules = rules;
        this.rules = rules.withGeometryEps(ROUTING_MARGIN_M);
        this.obstacles = new ObstacleSet(exact.all(), this.rules);
        this.router = new Router(obstacles, this.rules);
    }

    /**
     * Порядок добавления влияет на результат: ранние ветви становятся стволами и могут перекрыть путь позже идущим.
     * Цели, которые не удалось подключить не из-за здания, переносятся в начало порядка, и расчёт повторяется;
     * берётся решение с лучшим показателем (штраф за неподключённые уже в нём).
     */
    public Solution planBest(List<PlanInput.Target> order, int attempts, Router.Profile profile) {
        this.profile = profile;
        Solution best = plan(order);
        List<PlanInput.Target> current = order;
        for (int a = 1; a < attempts; a++) {
            List<PlanInput.Target> stuck = new ArrayList<>();
            for (Unconnected u : best.unconnected) {
                if (!u.startRefused) {
                    stuck.add(u.target);
                }
            }
            if (stuck.isEmpty()) {
                break;
            }
            List<PlanInput.Target> next = new ArrayList<>(stuck);
            for (PlanInput.Target t : current) {
                if (!stuck.contains(t)) {
                    next.add(t);
                }
            }
            current = next;
            Solution s = plan(next);
            if (s.score() < best.score()) {
                best = s;
            } else {
                break;
            }
        }
        return best;
    }

    public Solution plan(List<PlanInput.Target> order) {
        List<Branch> branches = new ArrayList<>();
        List<Unconnected> bad = new ArrayList<>();
        for (PlanInput.Target t : order) {
            long began = System.currentTimeMillis();
            Branch best = null;
            double bestScore = Double.MAX_VALUE;
            String reason = "маршрут не найден";
            List<String> reasons = new ArrayList<>();
            for (Branch candidate : candidates(t, branches, reasons)) {
                List<Branch> trial = new ArrayList<>(branches);
                trial.add(candidate);
                TreeEvaluator.Result r = TreeEvaluator.evaluate(in, exact, trial);
                if (debug) {
                    System.out.println("DBG t=" + t.id + " parent=" + candidate.parent + " len=" + Math.round(candidate.total) + " dn=" + candidate.route.dn + " feasible=" + r.feasible + (r.feasible ? " cost=" + Math.round(r.cost() / 1e6) + "M totalLen=" + Math.round(r.length) : " why=" + r.reason));
                }
                if (!r.feasible) {
                    reasons.add(r.reason);
                    continue;
                }
                double s = Rules.score(r.cost(), r.length);
                if (s < bestScore) {
                    bestScore = s;
                    best = candidate;
                }
            }
            if (debug) {
                System.out.println("DBG time t=" + t.id + " " + (System.currentTimeMillis() - began) + " ms");
            }
            if (best != null) {
                branches.add(best);
            } else {
                boolean refused = !reasons.isEmpty() && startRefusal(reasons.get(0));
                bad.add(new Unconnected(t, reasons.isEmpty() ? reason : reasons.get(0), refused));
            }
        }
        TreeEvaluator.Result r = TreeEvaluator.evaluate(in, exact, branches);
        return new Solution(branches, bad, r);
    }

    /** Поиск без запаса: нужен, когда выход от здания лежит в полосе запаса у чужого отступа. */
    private Router exactRouter() {
        if (exactRouter == null) {
            synchronized (this) {
                if (exactRouter == null) {
                    exactRouter = new Router(exact, exactRules);
                }
            }
        }
        return exactRouter;
    }

    private static boolean startRefusal(String reason) {
        return reason.startsWith("ближайшая") || reason.startsWith("выход") || reason.startsWith("точка подключения")
                || reason.startsWith("финальный") || reason.startsWith("направление");
    }

    // ---- Кандидаты ----------------------------------------------------------------------------------------------------

    private static final class Found {
        final Route route;
        final List<String> reasons;

        Found(Route route, List<String> reasons) {
            this.route = route;
            this.reasons = reasons;
        }
    }

    private List<Branch> candidates(PlanInput.Target t, List<Branch> branches, List<String> reasons) {
        List<Branch> out = new ArrayList<>();
        int first = Rules.indexForFlow(t.flow);
        if (first < 0) {
            reasons.add("расход " + t.flow + " т/ч больше пропускной способности самого большого ДУ");
            return out;
        }
        Geometry barrier = null;
        if (!branches.isEmpty()) {
            LineString[] lines = new LineString[branches.size()];
            for (int i = 0; i < lines.length; i++) {
                lines[i] = branches.get(i).line;
            }
            barrier = F.createMultiLineString(lines);
        }
        final Geometry wall = barrier;
        List<Future<Found>> tasks = new ArrayList<>();
        // 1. Существующая сеть.
        tasks.add(POOL.submit(() -> search(t, first, null, branches, wall)));
        // 2. Ближайшие уже построенные ветви.
        List<Integer> near = new ArrayList<>();
        for (int i = 0; i < branches.size(); i++) {
            near.add(i);
        }
        near.sort(Comparator.comparingDouble(i -> branches.get(i).line.distance(F.createPoint(t.xy))));
        double toNetwork = Double.MAX_VALUE;
        for (PlanInput.Segment s : in.segments) {
            toNetwork = Math.min(toNetwork, s.line.distance(F.createPoint(t.xy)));
        }
        int taken = 0;
        for (int i = 0; i < near.size() && taken < NEAREST_BRANCHES; i++) {
            Branch onto = branches.get(near.get(i));
            if (onto.line.distance(F.createPoint(t.xy)) > toNetwork * 1.5 + 150) {
                continue;
            }
            taken++;
            tasks.add(POOL.submit(() -> search(t, first, onto, branches, wall)));
        }
        for (Future<Found> f : tasks) {
            try {
                Found found = f.get();
                reasons.addAll(found.reasons);
                if (debug && found.route == null) {
                    System.out.println("DBG t=" + t.id + " no route: " + found.reasons);
                }
                addCandidate(out, t, found.route, branches);
            } catch (InterruptedException | ExecutionException e) {
                throw new IllegalStateException(e);
            }
        }
        return out;
    }

    /** Трасса цели к сети (onto == null) или к точкам одной ветви; ДУ растёт, пока трасса не уложится в предельную длину. */
    private Found search(PlanInput.Target t, int firstDn, Branch onto, List<Branch> branches, Geometry barrier) {
        List<String> reasons = new ArrayList<>();
        for (int i = firstDn; i < Math.min(Rules.DN.length, firstDn + MAX_DN_STEPS); i++) {
            int dn = Rules.DN[i];
            List<Start.Refusal> why = new ArrayList<>();
            ObstacleSet set = obstacles;
            Router rt = router;
            List<Start> starts = Start.forTarget(t, set, dn, rules, why);
            if (starts.isEmpty()) {
                // Запас мешает только выходу от здания: ищем без запаса, а итоговый диаметр проверит оценка дерева.
                why.clear();
                set = exact;
                rt = exactRouter();
                starts = Start.forTarget(t, set, dn, exactRules, why);
            }
            if (starts.isEmpty()) {
                if (!why.isEmpty()) {
                    reasons.add(why.get(0).message());
                }
                return new Found(null, reasons);
            }
            Route best = null;
            for (Start s : starts) {
                List<Goal> goals = onto == null ? Goals.forStart(in, set, s.origin, dn)
                        : branchGoals(onto, branches.indexOf(onto), s.origin);
                if (goals.isEmpty()) {
                    continue;
                }
                Router.Result r = rt.findRoute(s, goals, dn, profile, Rules.MAX_LENGTH[i], barrier);
                if (r.route != null && (best == null || r.route.cost < best.cost)) {
                    best = r.route;
                }
            }
            if (best != null) {
                return new Found(best, reasons);
            }
        }
        return new Found(null, reasons);
    }
    /** Цели поиска на обычных участках ветви: ближайшая к началу поиска точка и точки с шагом вдоль ветви. */
    private List<Goal> branchGoals(Branch b, int index, Coordinate origin) {
        List<Goal> goals = new ArrayList<>();
        LengthIndexedLine lil = new LengthIndexedLine(b.line);
        List<Double> positions = new ArrayList<>();
        positions.add(lil.project(origin));
        for (double d = BRANCH_GOAL_STEP_M / 2; d < b.total; d += BRANCH_GOAL_STEP_M) {
            positions.add(d);
        }
        for (double pos : positions) {
            if (b.canJoinAt(pos)) {
                Route.Leg leg = b.route.legs.get(b.legAt(pos));
                goals.add(new Goal(b.pointAt(pos), Collections.emptySet(), null, new Branch.Point(index), Angles.unit(leg.from, leg.to)));
            }
        }
        goals.sort(Comparator.comparingDouble(g -> g.xy.distance(origin)));
        return goals.subList(0, Math.min(goals.size(), MAX_BRANCH_GOALS));
    }

    /** Превращает найденную трассу в ветвь; при пересечении других ветвей обрезает трассу и присоединяет к первой из них. */
    private void addCandidate(List<Branch> out, PlanInput.Target t, Route route, List<Branch> branches) {
        if (route == null) {
            return;
        }
        Branch b = make(t, route, branches);
        if (b != null) {
            out.add(b);
        }
    }

    private Branch make(PlanInput.Target t, Route route, List<Branch> branches) {
        LineString line = lineOf(route);
        LengthIndexedLine lil = new LengthIndexedLine(line);
        int ownParent = route.goal.payload instanceof Branch.Point ? ((Branch.Point) route.goal.payload).branch : -1;
        double firstConflict = Double.MAX_VALUE;
        int conflictBranch = -1;
        Coordinate conflictPoint = null;
        for (int j = 0; j < branches.size(); j++) {
            Geometry inter = line.intersection(branches.get(j).line);
            if (inter.isEmpty()) {
                continue;
            }
            for (Coordinate c : inter.getCoordinates()) {
                boolean atOwnAttach = j == ownParent && c.distance(route.goal.xy) < 0.05;
                if (atOwnAttach) {
                    continue;
                }
                double pos = lil.project(c);
                if (pos < firstConflict) {
                    firstConflict = pos;
                    conflictBranch = j;
                    conflictPoint = c;
                }
            }
        }
        if (conflictBranch < 0) {
            if (ownParent >= 0) {
                return joinOnBranch(t, route, branches, ownParent, route.goal.xy);
            }
            return new Branch(t, route, -1, (Goals.Attach) route.goal.payload, route.goal.xy, 0);
        }
        // Трасса пересекает чужую ветвь: обрезаем её в первом пересечении и присоединяем там.
        Route cut = cutAt(route, firstConflict, conflictPoint, conflictBranch);
        return cut == null ? null : joinOnBranch(t, cut, branches, conflictBranch, conflictPoint);
    }

    private Branch joinOnBranch(PlanInput.Target t, Route route, List<Branch> branches, int parentIndex, Coordinate at) {
        Branch parent = branches.get(parentIndex);
        double pos = parent.project(at);
        if (!parent.canJoinAt(pos)) {
            return null;
        }
        // Поворот на узле: от последнего участка новой трассы к участку родительской ветви в сторону сети.
        Route.Leg last = route.legs.get(route.legs.size() - 1);
        Route.Leg onto = parent.route.legs.get(parent.legAt(pos));
        if (Angles.turnDeg(Angles.unit(last.from, last.to), Angles.unit(onto.from, onto.to)) > Rules.MAX_TURN_DEG + 1e-6) {
            return null;
        }
        return new Branch(t, route, parentIndex, null, at, pos);
    }

    private Route cutAt(Route route, double pos, Coordinate q, int parentIndex) {
        List<Route.Leg> legs = new ArrayList<>();
        double acc = 0;
        for (Route.Leg l : route.legs) {
            double len = l.length();
            if (acc + len < pos - 1e-9) {
                legs.add(l);
                acc += len;
                continue;
            }
            if (l.special || l.terminal || pos - acc < 0.5) {
                return null;
            }
            legs.add(new Route.Leg(l.from, q, false, 1, java.util.List.of(), false, false));
            Goal g = new Goal(q, Collections.emptySet(), null, new Branch.Point(parentIndex));
            return new Route(legs, route.start, g, route.dn);
        }
        return null;
    }

    private static LineString lineOf(Route r) {
        return F.createLineString(r.vertices().toArray(new Coordinate[0]));
    }
}