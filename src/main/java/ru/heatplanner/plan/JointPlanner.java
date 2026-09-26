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
    /** Самая большая перестраиваемая цепочка и число попыток за проход. */
    private static final int CHAIN_MAX_SIZE = 6;
    private static final int CHAIN_ATTEMPTS = 3;
    /** Поиск, обработавший не больше стольких меток и не нашедший пути, считается зажатым у старта. */
    private static final int CORNERED_LABELS = 50;
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

    public JointPlanner(PlanInput in, ObstacleSet exact, RuleSet rules) {
        this.in = in;
        this.exact = exact;
        this.exactRules = rules;
        this.rules = rules.withGeometryEps(ROUTING_MARGIN_M);
        this.obstacles = new ObstacleSet(exact.all(), this.rules, true);
        this.router = new Router(obstacles, this.rules);
    }

    /**
     * Порядок добавления влияет на результат: ранние ветви становятся стволами и могут перекрыть путь позже идущим.
     * Цели, которые не удалось подключить не из-за здания, переносятся в начало порядка, и расчёт повторяется;
     * берётся решение с лучшим показателем (штраф за неподключённые уже в нём).
     */
    public Solution planBest(List<PlanInput.Target> order, int attempts, Router.Profile profile) {
        Solution best = plan(order, profile);
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
            Solution s = plan(next, profile);
            if (s.score() < best.score()) {
                best = s;
            } else {
                break;
            }
        }
        return best;
    }

    /**
     * Улучшение готового решения: жадное добавление зависит от порядка целей, поэтому ранние ветви могут вырасти в
     * длинную цепочку. Каждая ветвь без потомков снимается и подключается заново ко всем вариантам (сеть и ближайшие
     * ветви); замена принимается, если показатель ранжирования всей сети улучшился. Повторяется несколько проходов:
     * освободившийся родитель на следующем проходе тоже становится концом цепочки.
     */
    public Solution improve(Solution start, Router.Profile profile, int passes) {
        Solution best = start;
        for (int pass = 0; pass < passes; pass++) {
            boolean changed = false;
            for (int k = best.branches.size() - 1; k >= 0; k--) {
                if (!isLeaf(best.branches, k)) {
                    continue;
                }
                List<Branch> rest = withoutBranch(best.branches, k);
                PlanInput.Target t = best.branches.get(k).target;
                double bestScore = best.score();
                Branch replacement = null;
                for (Branch candidate : candidates(t, rest, new ArrayList<>(), profile)) {
                    List<Branch> trial = new ArrayList<>(rest);
                    trial.add(candidate);
                    TreeEvaluator.Result r = TreeEvaluator.evaluate(in, exact, trial);
                    if (!r.feasible) {
                        continue;
                    }
                    double s = Rules.score(r.cost() + best.penalty, r.length);
                    if (s < bestScore - 1e-6) {
                        bestScore = s;
                        replacement = candidate;
                    }
                }
                if (replacement != null) {
                    List<Branch> next = new ArrayList<>(rest);
                    next.add(replacement);
                    best = new Solution(next, best.unconnected, TreeEvaluator.evaluate(in, exact, next));
                    changed = true;
                    if (debug) {
                        System.out.println("DBG improve moved t=" + t.id + " score=" + best.score());
                    }
                }
            }
            if (!changed) {
                break;
            }
        }
        return best;
    }

    private static boolean isLeaf(List<Branch> bs, int k) {
        for (Branch b : bs) {
            if (b.parent == k) {
                return false;
            }
        }
        return true;
    }

    /** Список без ветви k: номера родителей после неё сдвигаются. */
    static List<Branch> withoutBranch(List<Branch> bs, int k) {
        List<Branch> out = new ArrayList<>();
        for (int i = 0; i < bs.size(); i++) {
            if (i == k) {
                continue;
            }
            Branch b = bs.get(i);
            int parent = b.parent > k ? b.parent - 1 : b.parent;
            out.add(parent == b.parent ? b : new Branch(b.target, b.route, parent, b.existing, b.attach, b.parentPos));
        }
        return out;
    }

    public Solution plan(List<PlanInput.Target> order, Router.Profile profile) {
        return insertAll(new ArrayList<>(), new ArrayList<>(), order, profile);
    }

    /** Добавляет цели по очереди к уже построенным ветвям base; не подключённые попадают в список bad. */
    private Solution insertAll(List<Branch> base, List<Unconnected> lostBefore, List<PlanInput.Target> order, Router.Profile profile) {
        List<Branch> branches = new ArrayList<>(base);
        List<Unconnected> bad = new ArrayList<>(lostBefore);
        for (PlanInput.Target t : order) {
            long began = System.currentTimeMillis();
            Branch best = null;
            double bestScore = Double.MAX_VALUE;
            String reason = "маршрут не найден";
            List<String> reasons = new ArrayList<>();
            for (Branch candidate : candidates(t, branches, reasons, profile)) {
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

    /**
     * Перестройка цепочек: ветвь вместе со всеми, кто к ней (прямо или через другие) примыкает, снимается и собирается
     * заново в другом порядке (сначала дальние от сети, сначала ближние). Так меняются роли внутри цепочки: ствол
     * может пойти от другого дома. Замена принимается, если показатель S всей сети улучшился.
     */
    public Solution rebuildChains(Solution start, Router.Profile profile) {
        Solution best = start;
        boolean changed = true;
        int rounds = 0;
        while (changed && rounds++ < 1) {
            changed = false;
            // Пробуем самые большие группы (цепочки): их перестройка меняет структуру сильнее всего.
            List<Integer> roots = new ArrayList<>();
            for (int k = 0; k < best.branches.size(); k++) {
                int size = subtree(best.branches, k).size();
                if (size >= 2 && size <= CHAIN_MAX_SIZE) {
                    roots.add(k);
                }
            }
            final Solution current = best;
            roots.sort(Comparator.comparingInt((Integer k) -> -subtree(current.branches, k).size()));
            for (int attempt = 0; attempt < Math.min(CHAIN_ATTEMPTS, roots.size()); attempt++) {
                int k = roots.get(attempt);
                List<Integer> group = subtree(best.branches, k);
                List<Branch> rest = new ArrayList<>();
                List<PlanInput.Target> targets = new ArrayList<>();
                for (int i = 0; i < best.branches.size(); i++) {
                    if (!group.contains(i)) {
                        rest.add(best.branches.get(i));
                    } else {
                        targets.add(best.branches.get(i).target);
                    }
                }
                rest = renumber(best.branches, group);
                List<List<PlanInput.Target>> orders = new ArrayList<>();
                List<PlanInput.Target> far = new ArrayList<>(targets);
                far.sort(Comparator.comparingDouble((PlanInput.Target t) -> -Variants.distanceToNetwork(in, t)));
                List<PlanInput.Target> near = new ArrayList<>(far);
                Collections.reverse(near);
                orders.add(far);
                orders.add(near);
                Solution improved = null;
                for (List<PlanInput.Target> order : orders) {
                    Solution s = insertAll(rest, best.unconnected, order, profile);
                    if (s.unconnected.size() <= best.unconnected.size() && s.score() < (improved == null ? best.score() : improved.score()) - 1e-6) {
                        improved = s;
                    }
                }
                if (improved != null) {
                    best = improved;
                    changed = true;
                    if (debug) {
                        System.out.println("DBG chain rebuilt at t=" + best.branches.get(Math.min(k, best.branches.size() - 1)).target.id + " score=" + best.score());
                    }
                    break; // индексы сменились: начинаем обход заново
                }
            }
        }
        return best;
    }

    /** Номера ветви k и всех её потомков. */
    private static List<Integer> subtree(List<Branch> bs, int k) {
        List<Integer> out = new ArrayList<>();
        out.add(k);
        for (int i = k + 1; i < bs.size(); i++) {
            if (out.contains(bs.get(i).parent)) {
                out.add(i);
            }
        }
        return out;
    }

    /** Ветви без указанной группы: родители переименовываются по новым номерам (группа — замкнутое поддерево). */
    private static List<Branch> renumber(List<Branch> bs, List<Integer> removed) {
        int[] shift = new int[bs.size()];
        int gone = 0;
        for (int i = 0; i < bs.size(); i++) {
            shift[i] = i - gone;
            if (removed.contains(i)) {
                gone++;
            }
        }
        List<Branch> out = new ArrayList<>();
        for (int i = 0; i < bs.size(); i++) {
            if (removed.contains(i)) {
                continue;
            }
            Branch b = bs.get(i);
            int parent = b.parent < 0 ? -1 : shift[b.parent];
            out.add(parent == b.parent ? b : new Branch(b.target, b.route, parent, b.existing, b.attach, b.parentPos));
        }
        return out;
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

    List<Branch> candidates(PlanInput.Target t, List<Branch> branches, List<String> reasons, Router.Profile profile) {
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
        tasks.add(POOL.submit(() -> search(t, first, null, branches, wall, profile)));
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
            tasks.add(POOL.submit(() -> search(t, first, onto, branches, wall, profile)));
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

    /** Итог поиска для одного диаметра. */
    private static final class Outcome {
        final Route best;
        /** Пути отбрасывались из-за предельной длины: с большим диаметром маршрут мог бы найтись. */
        final boolean limited;
        /** К точке нельзя подойти от здания вовсе. */
        final boolean noStart;
        final String reason;

        Outcome(Route best, boolean limited, boolean noStart, String reason) {
            this.best = best;
            this.limited = limited;
            this.noStart = noStart;
            this.reason = reason;
        }
    }

    /**
     * Поиск к сети без преграды не зависит от уже построенных ветвей, поэтому его результат общий для всех стратегий и
     * шагов: если такого пути нет, то с преградой его тем более нет, а если он есть и не задевает преграду, то он и
     * оптимален для неё.
     */
    private final java.util.concurrent.ConcurrentHashMap<String, Outcome> freeMemo = new java.util.concurrent.ConcurrentHashMap<>();

    /** Трасса цели к сети (onto == null) или к точкам одной ветви; ДУ растёт, пока трасса не уложится в предельную длину. */
    private Found search(PlanInput.Target t, int firstDn, Branch onto, List<Branch> branches, Geometry barrier, Router.Profile profile) {
        List<String> reasons = new ArrayList<>();
        for (int i = firstDn; i < Math.min(Rules.DN.length, firstDn + MAX_DN_STEPS); i++) {
            Outcome o;
            if (onto == null) {
                String key = t.id.value() + "|" + t.id.isNumeric() + "|" + i + "|" + profile;
                Outcome free = freeMemo.get(key);
                if (free == null) {
                    free = attempt(t, i, null, branches, null, profile);
                    freeMemo.put(key, free);
                }
                if (free.noStart || (free.best == null && !free.limited)) {
                    o = free;
                } else if (free.best != null && (barrier == null || !lineOf(free.best).intersects(barrier))) {
                    o = free;
                } else {
                    o = attempt(t, i, null, branches, barrier, profile);
                }
            } else {
                o = attempt(t, i, onto, branches, barrier, profile);
            }
            if (o.reason != null) {
                reasons.add(o.reason);
            }
            if (o.best != null) {
                return new Found(o.best, reasons);
            }
            if (o.noStart || !o.limited) {
                // Пути нет не из-за предельной длины: больший диаметр даёт только большие отступы, маршрут не появится.
                return new Found(null, reasons);
            }
        }
        return new Found(null, reasons);
    }

    /** Подходы к цели и причина отказа для одного диаметра и набора ограничений: общие для всех стратегий. */
    private static final class Starts {
        final List<Start> list;
        final String reason;
        final boolean exactSet;

        Starts(List<Start> list, String reason, boolean exactSet) {
            this.list = list;
            this.reason = reason;
            this.exactSet = exactSet;
        }
    }

    private final java.util.concurrent.ConcurrentHashMap<String, Starts> startsMemo = new java.util.concurrent.ConcurrentHashMap<>();
    private final java.util.concurrent.ConcurrentHashMap<String, List<Goal>> goalsMemo = new java.util.concurrent.ConcurrentHashMap<>();

    private Starts startsFor(PlanInput.Target t, int dn) {
        String key = t.id.value() + "|" + t.id.isNumeric() + "|" + dn;
        return startsMemo.computeIfAbsent(key, k -> {
            List<Start.Refusal> why = new ArrayList<>();
            List<Start> starts = Start.forTarget(t, obstacles, dn, rules, why);
            boolean nearestOk = false;
            for (Start s : starts) {
                nearestOk |= s.nearestBoundary;
            }
            if (nearestOk) {
                return new Starts(starts, null, false);
            }
            List<Start> withoutMargin = Start.forTarget(t, exact, dn, exactRules, new ArrayList<>());
            boolean exactNearest = false;
            for (Start s : withoutMargin) {
                exactNearest |= s.nearestBoundary;
            }
            if (!starts.isEmpty() && !exactNearest) {
                return new Starts(starts, null, false);
            }
            if (exactNearest) {
                return new Starts(withoutMargin, null, true);
            }
            // Запас мешает только выходу от здания: ищем без запаса, а итоговый диаметр проверит оценка дерева.
            why.clear();
            starts = Start.forTarget(t, exact, dn, exactRules, why);
            return new Starts(starts, why.isEmpty() ? null : why.get(0).message(), true);
        });
    }

    private List<Goal> networkGoals(ObstacleSet set, Coordinate origin, int dn) {
        String key = (set == exact ? "e" : "m") + "|" + dn + "|" + Math.round(origin.x * 100) + "|" + Math.round(origin.y * 100);
        return goalsMemo.computeIfAbsent(key, k -> Goals.forStart(in, set, origin, dn));
    }

    /** Подходы к цели без запаса (только для исключений: запас зажал старт). */
    private final java.util.concurrent.ConcurrentHashMap<String, List<Start>> exactStartsMemo = new java.util.concurrent.ConcurrentHashMap<>();

    private Outcome attempt(PlanInput.Target t, int i, Branch onto, List<Branch> branches, Geometry barrier, Router.Profile profile) {
        int dn = Rules.DN[i];
        Starts found = startsFor(t, dn);
        if (found.list.isEmpty()) {
            return new Outcome(null, false, true, found.reason);
        }
        boolean[] cornered = new boolean[1];
        Outcome o = run(t, i, found.list, found.exactSet ? exact : obstacles, found.exactSet ? exactRouter() : router,
                onto, branches, barrier, profile, cornered);
        if (o.best == null && cornered[0] && !found.exactSet) {
            // Поиск умер у самого старта: запас у чужого отступа не оставил выхода. Повторяем без запаса,
            // а итоговый диаметр потом проверит оценка дерева.
            String key = t.id.value() + "|" + t.id.isNumeric() + "|" + dn;
            List<Start> exactStarts = exactStartsMemo.computeIfAbsent(key,
                    k -> Start.forTarget(t, exact, dn, exactRules, new ArrayList<>()));
            if (!exactStarts.isEmpty()) {
                Outcome again = run(t, i, exactStarts, exact, exactRouter(), onto, branches, barrier, profile, new boolean[1]);
                if (again.best != null) {
                    return again;
                }
            }
        }
        return o;
    }

    private Outcome run(PlanInput.Target t, int i, List<Start> starts, ObstacleSet set, Router rt, Branch onto, List<Branch> branches,
                        Geometry barrier, Router.Profile profile, boolean[] cornered) {
        int dn = Rules.DN[i];
        Route best = null;
        boolean limited = false;
        // Запасной подход (к зданию, к которому по правилу не подойти): берётся ближайшая возможная граница, а не
        // самая выгодная из нескольких; следующая по расстоянию пробуется только если от этой маршрута нет.
        boolean sequential = !starts.isEmpty() && !starts.get(0).nearestBoundary;
        for (Start s : starts) {
            if (sequential && best != null) {
                break;
            }
            List<Goal> goals = onto == null ? networkGoals(set, s.origin, dn)
                    : branchGoals(onto, branches.indexOf(onto), s.origin, branches);
            if (goals.isEmpty()) {
                continue;
            }
            long began = System.currentTimeMillis();
            Router.Result r = rt.findRoute(s, goals, dn, profile, Rules.MAX_LENGTH[i], barrier);
            if (debug) {
                System.out.println("DBG search t=" + t.id + " onto=" + (onto == null ? "net" : onto.target.id.value()) + " dn=" + dn
                        + " goals=" + goals.size() + " " + (System.currentTimeMillis() - began) + " ms " + r.status + " labels=" + r.labels);
            }
            limited |= r.lengthLimited;
            if (r.route == null && r.labels <= CORNERED_LABELS) {
                cornered[0] = true;
            }
            if (r.route != null && (best == null || r.route.cost < best.cost)) {
                best = r.route;
            }
        }
        return new Outcome(best, limited, false, null);
    }
    /** Цели поиска на обычных участках ветви: ближайшая к началу поиска точка и точки с шагом вдоль ветви. */
    private List<Goal> branchGoals(Branch b, int index, Coordinate origin, List<Branch> all) {
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
        List<Goal> limited = new ArrayList<>(goals.subList(0, Math.min(goals.size(), MAX_BRANCH_GOALS)));
        // Уже существующие узлы на ветви: вторая ветвь может закончиться в той же камере (четвёртое примыкание).
        for (Branch child : all) {
            if (child.parent == index && b.canJoinAt(child.parentPos)) {
                Route.Leg leg = b.route.legs.get(b.legAt(child.parentPos));
                limited.add(new Goal(child.attach, Collections.emptySet(), null, new Branch.Point(index), Angles.unit(leg.from, leg.to)));
            }
        }
        return limited;
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
                boolean sibling = ownParent >= 0 && j != ownParent && branches.get(j).parent == ownParent
                        && branches.get(j).attach.distance(route.goal.xy) < 0.05;
                boolean atOwnAttach = (j == ownParent || sibling) && c.distance(route.goal.xy) < 0.05;
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