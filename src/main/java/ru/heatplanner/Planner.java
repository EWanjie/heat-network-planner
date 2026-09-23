package ru.heatplanner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

/**
 * Поиск вариантов подключения. Общая схема одного варианта:
 * 1. ОКС берутся по очереди, самые удалённые от сети первыми, чтобы ближние потом присоединялись к уже
 *    проложенным трубам (совместное подключение).
 * 2. Для ОКС ищутся все кратчайшие пути с учётом запретов (алгоритм Дейкстры по растровой карте), из них выбираются
 *    кандидаты: врезка в существующую камеру или участок, либо присоединение к новой трубе (новая камера-развилка).
 * 3. Каждый кандидат оценивается полностью (Evaluator): диаметры, камеры, врезки, реконструкция, штраф.
 *    Выбирается тот, что лучше по цели варианта. Если подключение дороже штрафа, ОКС остаётся неподключённым.
 * 4. Затем проходы «снять и подключить заново»: каждый ОКС переподключается с учётом остальных, пока выгодно.
 *
 * Варианты отличаются целью (профилем): критерий ТЗ, минимум вмешательства в существующую сеть,
 * кратчайшие трассы, минимальная стоимость. Одинаковые по существу варианты отбрасываются.
 */
final class Planner {

    /** Цель варианта: веса оценки. Итоговое ранжирование всё равно идёт по показателю S из ТЗ. */
    static final class Profile {
        final String id;
        final String name;
        final String idea;
        final double wCost;
        final double wLength;
        /** Дополнительный вес стоимости реконструкции существующей сети. */
        final double wRecon;
        /** Штраф за каждую врезку в единицах оценки. */
        final double wTie;

        Profile(String id, String name, String idea, double wCost, double wLength, double wRecon, double wTie) {
            this.id = id;
            this.name = name;
            this.idea = idea;
            this.wCost = wCost;
            this.wLength = wLength;
            this.wRecon = wRecon;
            this.wTie = wTie;
        }
    }

    static final Profile BALANCED = new Profile("balanced", "Оптимум по критерию ТЗ",
            "Минимизирует итоговый показатель из ТЗ: 70 % стоимость и 30 % протяжённость.", 0.7, 0.3, 0, 0);
    static final Profile LOW_IMPACT = new Profile("low_impact", "Минимум вмешательства в существующую сеть",
            "Реже врезается и меньше перестраивает существующие трубы: главные магистрали длиннее, зато сеть менее нагружена.",
            0.7, 0.3, 2.0, 0.3);
    static final Profile SHORTEST = new Profile("shortest", "Кратчайшие трассы",
            "Минимизирует протяжённость линейных работ, стоимость второстепенна.", 0.02, 1.0, 0, 0);
    static final Profile CHEAPEST = new Profile("cheapest", "Минимальная стоимость",
            "Минимизирует только итоговую стоимость.", 1.0, 0, 0, 0);
    /** Во сколько раз штраф за неподключённый ОКС весит больше обычного рубля при поиске (не влияет на итоговый S). */
    private static final double PENALTY_WEIGHT = 3.0;

    /** Порядок важен: из одинаковых по существу вариантов остаётся первый. */
    private static final Profile[] PROFILES = {BALANCED, LOW_IMPACT, SHORTEST, CHEAPEST};

    /** Один рассчитанный вариант со всем, что нужно для вывода и объяснения. */
    static final class Variant {
        Profile profile;
        Tree tree;
        Solution solution;
        /** Причина для каждого неподключённого ОКС. */
        Map<PlanModel.Target, String> reasons = new LinkedHashMap<>();
        int rank;
    }

    static final class Result {
        PlanModel model;
        RasterMap map;
        /** Варианты по возрастанию показателя S (первый — лучший). */
        List<Variant> variants = new ArrayList<>();
        /** Тот же алгоритм, но каждый ОКС подключается отдельно, без совместных труб: база для оценки экономии. */
        Variant separate;
        List<String> notes = new ArrayList<>();
    }

    private final PlanModel model;
    private final RasterMap map;

    private Planner(PlanModel model, RasterMap map) {
        this.model = model;
        this.map = map;
    }

    static Result solve(PlanModel model) throws InterruptedException, ExecutionException {
        if (model.sources.isEmpty()) {
            throw new IllegalArgumentException("В файле нет источника теплоснабжения (source).");
        }
        if (model.segments.isEmpty()) {
            throw new IllegalArgumentException("В файле нет существующих участков тепловой сети (heat_network).");
        }
        if (model.targets.isEmpty()) {
            throw new IllegalArgumentException("В файле нет перспективных ОКС и точек их подключения.");
        }
        Result result = new Result();
        result.model = model;
        result.map = RasterMap.build(model);
        Planner planner = new Planner(model, result.map);

        // Профили и «отдельное подключение» считаются параллельно: у каждого свой Router.
        ExecutorService pool = Executors.newFixedThreadPool(Math.min(PROFILES.length + 1,
                Math.max(2, Runtime.getRuntime().availableProcessors())));
        try {
            List<Future<Variant>> futures = new ArrayList<>();
            for (Profile p : PROFILES) {
                futures.add(pool.submit(() -> planner.run(p, true)));
            }
            Future<Variant> separate = pool.submit(() -> planner.run(BALANCED, false));
            List<Variant> all = new ArrayList<>();
            for (Future<Variant> f : futures) {
                all.add(f.get());
            }
            result.separate = separate.get();

            // Вторая фаза: каждый профиль стартует с лучшего по его цели дерева среди всех найденных и доводится.
            List<Future<Variant>> polished = new ArrayList<>();
            for (int i = 0; i < PROFILES.length; i++) {
                Profile p = PROFILES[i];
                polished.add(pool.submit(() -> {
                    Run run = planner.new Run(p, true);
                    Tree seed = null;
                    double best = Double.MAX_VALUE;
                    for (Variant v : all) {
                        double value = run.value(v.solution);
                        if (seed == null || value < best) {
                            best = value;
                            seed = v.tree;
                        }
                    }
                    return run.polish(seed);
                }));
            }
            List<Variant> refined = new ArrayList<>();
            for (Future<Variant> f : polished) {
                refined.add(f.get());
            }
            result.variants = pickDistinct(refined, result.notes);
        } finally {
            pool.shutdown();
        }
        result.variants.sort((a, b) -> Double.compare(a.solution.score(), b.solution.score()));
        for (int i = 0; i < result.variants.size(); i++) {
            result.variants.get(i).rank = i + 1;
        }
        return result;
    }

    /** До трёх существенно различающихся вариантов; повторы отбрасываются с пояснением. */
    private static List<Variant> pickDistinct(List<Variant> all, List<String> notes) {
        List<Variant> kept = new ArrayList<>();
        for (Variant v : all) {
            Variant twin = null;
            for (Variant k : kept) {
                if (similar(k, v)) {
                    twin = k;
                    break;
                }
            }
            if (twin != null) {
                notes.add("Вариант «" + v.profile.name + "» совпал по существу с вариантом «" + twin.profile.name
                        + "» и не показан.");
            } else if (kept.size() < 3) {
                kept.add(v);
            }
        }
        return kept;
    }

    /** Варианты существенно совпадают: та же длина (±3 %), то же число врезок и врезки в тех же местах (±10 м). */
    private static boolean similar(Variant a, Variant b) {
        Solution x = a.solution;
        Solution y = b.solution;
        if (x.tieIns.size() != y.tieIns.size() || x.unconnected.size() != y.unconnected.size()) {
            return false;
        }
        double lx = x.totalLength();
        double ly = y.totalLength();
        if (Math.abs(lx - ly) > 0.03 * Math.max(lx, ly)) {
            return false;
        }
        for (Solution.TieIn t : x.tieIns) {
            boolean found = false;
            for (Solution.TieIn u : y.tieIns) {
                if (Geo.dist(t.node.xy, u.node.xy) <= 10) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                return false;
            }
        }
        return true;
    }

    // ---- Один вариант ---------------------------------------------------------------------------------------------

    private Variant run(Profile profile, boolean join) {
        return new Run(profile, join).execute();
    }

    private static final class Cand {
        static final int SEGMENT = 0;
        static final int CHAMBER = 1;
        static final int NODE = 2;
        static final int EDGE = 3;
        int kind;
        PlanModel.Segment segment;
        double along;
        PlanModel.Chamber chamber;
        int nodeId;
        int edgeIndex;
        double[] xy;
        double proxy;
    }

    private final class Run {
        final Profile profile;
        final boolean join;
        final Router router = new Router(map);
        final Map<PlanModel.Target, String> reasons = new HashMap<>();

        Run(Profile profile, boolean join) {
            this.profile = profile;
            this.join = join;
        }

        /** Первая фаза: жадная сборка сети ОКС за ОКС и проходы улучшения. */
        Variant execute() {
            List<PlanModel.Target> order = order();
            Tree tree = new Tree();
            for (PlanModel.Target t : order) {
                tree = insert(tree, t);
            }
            return finish(improve(tree, order, 2));
        }

        /** Вторая фаза: доводка готового дерева проходами «снять и подключить заново» по цели этого варианта. */
        Variant polish(Tree seed) {
            List<PlanModel.Target> order = order();
            return finish(improve(seed.copy(), order, 3));
        }

        List<PlanModel.Target> order() {
            List<PlanModel.Target> order = new ArrayList<>();
            for (PlanModel.Target t : model.targets) {
                if (t.problem == null) {
                    order.add(t);
                } else {
                    reasons.put(t, t.problem);
                }
            }
            // Самые удалённые от сети первыми: ближние потом присоединятся к их трубам.
            order.sort((a, b) -> Double.compare(distanceToNetwork(b), distanceToNetwork(a)));
            return order;
        }

        /**
         * Проходы улучшения: каждый ОКС снимается и подключается заново с учётом остальных; результат принимается,
         * только если оценка стала лучше. Неподключённые ОКС на каждом проходе пробуются снова.
         */
        Tree improve(Tree tree, List<PlanModel.Target> order, int passes) {
            for (int pass = 0; pass < passes; pass++) {
                boolean improved = false;
                for (PlanModel.Target t : order) {
                    double before = value(Evaluator.evaluate(tree, model));
                    Tree without = tree.copy();
                    without.remove(t, model);
                    Tree again = insert(without, t);
                    if (again.contains(t) && value(Evaluator.evaluate(again, model)) < before - 1e-9) {
                        tree = again;
                        improved = true;
                    }
                }
                if (!improved) {
                    break;
                }
            }
            return tree;
        }

        Variant finish(Tree tree) {
            Variant v = new Variant();
            v.profile = profile;
            v.tree = tree;
            v.solution = Evaluator.evaluate(tree, model);
            for (PlanModel.Target t : v.solution.unconnected) {
                v.reasons.put(t, reasons.getOrDefault(t, "подключение дороже штрафа за неподключённый ОКС"));
            }
            return v;
        }
        double value(Solution s) {
            if (s.invalid) {
                return Double.MAX_VALUE;
            }
            // Штраф за неподключённый ОКС весит втрое больше обычной стоимости: ОКС остаётся без подключения
            // только когда путь невозможен или подключение несоразмерно дороже штрафа. Итоговый S считается по ТЗ отдельно.
            return profile.wCost * (s.totalCost() - s.penalty) / Rules.COST_SCALE
                    + PENALTY_WEIGHT * s.penalty / Rules.COST_SCALE
                    + profile.wLength * s.totalLength() / Rules.LENGTH_SCALE
                    + profile.wRecon * (s.reconCost + s.chamberReconCost) / Rules.COST_SCALE
                    + profile.wTie * s.tieIns.size();
        }

        double distanceToNetwork(PlanModel.Target t) {
            double best = Double.MAX_VALUE;
            for (PlanModel.ConnectionPoint p : t.points) {
                for (PlanModel.Segment s : model.segments.values()) {
                    best = Math.min(best, Geo.project(s.pts, p.xy[0], p.xy[1])[1]);
                }
            }
            return best;
        }

        /**
         * Подключает ОКС к сети лучшим из способов и возвращает новое дерево;
         * если выгоднее оставить ОКС неподключённым (или пути нет), возвращает исходное.
         */
        Tree insert(Tree tree, PlanModel.Target target) {
            Solution base = Evaluator.evaluate(tree, model);
            double best = value(base);
            Tree bestTree = tree;
            Solution cheapestConnection = null;
            boolean anyPath = false;
            String problem = null;
            for (PlanModel.ConnectionPoint cp : target.points) {
                Exempt ex = exempt(cp);
                if (ex.problem != null) {
                    problem = ex.problem;
                    continue;
                }
                router.clearOccupancy();
                for (Tree.Edge e : tree.edges) {
                    router.occupy(e.pts);
                }
                if (!router.begin(cp.xy, ex.radius, ex.own, model)) {
                    problem = "точка подключения " + cp.id + " вне расчётной области";
                    continue;
                }
                router.run();
                for (Cand c : candidates(tree, target)) {
                    double[][] pts = router.path(cp.xy, c.xy, true);
                    if (pts == null) {
                        continue;
                    }
                    if (!valid(pts, tree, cp, ex)) {
                        pts = router.path(cp.xy, c.xy, false);
                        if (pts == null || !valid(pts, tree, cp, ex)) {
                            continue;
                        }
                    }
                    anyPath = true;
                    Tree t2 = attach(tree, c, pts, target, cp, ex);
                    if (t2 == null) {
                        continue;
                    }
                    Solution s2 = Evaluator.evaluate(t2, model);
                    double v = value(s2);
                    if (!s2.invalid && (cheapestConnection == null || s2.totalCost() < cheapestConnection.totalCost())) {
                        cheapestConnection = s2;
                    }
                    if (v < best) {
                        best = v;
                        bestTree = t2;
                    }
                }
                router.restore();
            }
            if (bestTree == tree) {
                String why;
                if (problem != null) {
                    why = problem;
                } else if (cheapestConnection != null) {
                    // Подключение возможно, но несоразмерно дорого: показываем цифры, чтобы решение было проверяемым.
                    double extra = cheapestConnection.totalCost() - base.totalCost() + Rules.penalty(target.flow);
                    double extraLength = cheapestConnection.totalLength() - base.totalLength();
                    why = String.format(java.util.Locale.ROOT, "подключение возможно (около %.0f млн руб. и %.0f м работ), "
                            + "но невыгодно по сравнению со штрафом %.0f млн руб.", extra / 1e6, extraLength,
                            Rules.penalty(target.flow) / 1e6);
                } else {
                    why = anyPath ? "путь найден, но расчёт диаметров невозможен (расход больше пропускной способности)"
                            : "нет допустимого пути: ОКС окружён запретными зонами либо сеть недостижима";
                }
                reasons.put(target, why);
            } else {
                reasons.remove(target);
            }
            return bestTree;
        }

        /** Что не действует у точки подключения: собственное здание ОКС и зазор рядом с точкой. */
        final class Exempt {
            /** Радиус вокруг точки, где зазор до запретных зон не проверяется (точка часто стоит у самого здания). */
            double radius;
            /** Контуры существующих ОКС, внутри которых стоит точка: это здание самого ОКС, зазор к нему не применяется. */
            final List<PlanModel.Shape> own = new ArrayList<>();
            /** Причина, по которой подключить нельзя (точка в запретной зоне другого типа). */
            String problem;
        }

        Exempt exempt(PlanModel.ConnectionPoint cp) {
            Exempt ex = new Exempt();
            for (PlanModel.Shape s : model.shapes) {
                if (!s.rule.forbidden) {
                    continue;
                }
                double pad = s.rule.minDistance + 2 * map.cell;
                if (cp.xy[0] < s.minX - pad || cp.xy[0] > s.maxX + pad || cp.xy[1] < s.minY - pad || cp.xy[1] > s.maxY + pad) {
                    continue;
                }
                boolean in = s.area && Geo.inside(cp.xy[0], cp.xy[1], s.rings);
                double boundary = Geo.pointBoundary(cp.xy[0], cp.xy[1], s.rings);
                if (in && "oks".equals(s.rule.type)) {
                    ex.own.add(s);
                } else if (in && boundary > 2.5) {
                    ex.problem = "точка подключения " + cp.id + " находится внутри запретной зоны «" + s.rule.title + "»";
                    return ex;
                } else if (in || boundary < s.rule.minDistance + map.cell) {
                    ex.radius = Math.max(ex.radius, s.rule.minDistance + 2 * map.cell + (in ? boundary : 0));
                }
            }
            return ex;
        }

        /** Кандидаты подключения с приблизительной стоимостью, лучшие в начале. */
        List<Cand> candidates(Tree tree, PlanModel.Target target) {
            double perMeter = Rules.COST_NEW[Rules.indexForFlow(target.flow)];
            List<Cand> existing = new ArrayList<>();
            List<Cand> shared = new ArrayList<>();

            for (PlanModel.Chamber ch : model.chambers.values()) {
                float cost = router.costAt(ch.xy[0], ch.xy[1]);
                if (!Router.reachable(cost)) {
                    continue;
                }
                Tree.Node tie = tree.findTie(ch);
                if (tie != null) {
                    if (join && tie.canTakeBranch()) {
                        shared.add(nodeCand(tie, cost * perMeter));
                    }
                } else if (ch.adjacent < Rules.MAX_CHAMBER_SEGMENTS) {
                    Cand c = new Cand();
                    c.kind = Cand.CHAMBER;
                    c.chamber = ch;
                    c.xy = ch.xy;
                    c.proxy = cost * perMeter + Rules.TIE_IN_COST + reconProxy(ch.upSeg, target.flow, 0);
                    existing.add(c);
                }
            }
            double step = Math.max(map.cell * 2, 2.0);
            for (PlanModel.Segment s : model.segments.values()) {
                Cand best = null;
                for (double d = 0; d <= s.length + 1e-9; d += step) {
                    double[] p = Geo.pointAt(s.pts, Math.min(d, s.length));
                    float cost = router.costAt(p[0], p[1]);
                    if (!Router.reachable(cost) || nearChamber(p)) {
                        continue;
                    }
                    double proxy = cost * perMeter + Rules.TIE_IN_COST + Rules.chamberCost(s.dn)
                            + reconProxy(s, target.flow, Math.min(d, s.length));
                    if (best == null || proxy < best.proxy) {
                        best = new Cand();
                        best.kind = Cand.SEGMENT;
                        best.segment = s;
                        best.along = Math.min(d, s.length);
                        best.xy = p;
                        best.proxy = proxy;
                    }
                }
                if (best != null) {
                    existing.add(best);
                }
            }
            if (join) {
                for (Tree.Node n : tree.nodes) {
                    if (n.kind == Tree.Kind.JUNCTION && n.canTakeBranch()) {
                        float cost = router.costAt(n.xy[0], n.xy[1]);
                        if (Router.reachable(cost)) {
                            shared.add(nodeCand(n, cost * perMeter));
                        }
                    }
                }
                for (int i = 0; i < tree.edges.size(); i++) {
                    Tree.Edge e = tree.edges.get(i);
                    Cand best = null;
                    for (double d = 1; d < e.length - 1; d += step) {
                        double[] p = Geo.pointAt(e.pts, d);
                        float cost = router.costAt(p[0], p[1]);
                        if (!Router.reachable(cost)) {
                            continue;
                        }
                        double proxy = cost * perMeter + Rules.chamberCost(200);
                        if (best == null || proxy < best.proxy) {
                            best = new Cand();
                            best.kind = Cand.EDGE;
                            best.edgeIndex = i;
                            best.along = d;
                            best.xy = p;
                            best.proxy = proxy;
                        }
                    }
                    if (best != null) {
                        shared.add(best);
                    }
                }
            }
            existing.sort((a, b) -> Double.compare(a.proxy, b.proxy));
            shared.sort((a, b) -> Double.compare(a.proxy, b.proxy));
            List<Cand> out = new ArrayList<>(existing.subList(0, Math.min(10, existing.size())));
            out.addAll(shared.subList(0, Math.min(8, shared.size())));
            return out;
        }

        Cand nodeCand(Tree.Node n, double proxy) {
            Cand c = new Cand();
            c.kind = Cand.NODE;
            c.nodeId = n.id;
            c.xy = n.xy;
            c.proxy = proxy;
            return c;
        }

        /** Точка ближе 10 м к камере, в которую ещё можно врезаться: по правилу 8.2 врезка идёт в камеру. */
        boolean nearChamber(double[] p) {
            for (PlanModel.Chamber ch : model.chambers.values()) {
                if (ch.adjacent < Rules.MAX_CHAMBER_SEGMENTS && Geo.dist(ch.xy, p) <= Rules.CHAMBER_SNAP_M) {
                    return true;
                }
            }
            return false;
        }

        /** Грубая стоимость реконструкции от точки врезки к источнику для расхода flow (без учёта других ОКС). */
        double reconProxy(PlanModel.Segment first, double flow, double partial) {
            double cost = 0;
            boolean firstPartial = partial > 0;
            for (PlanModel.Segment s = first; s != null; s = s.upSeg) {
                int req = Rules.indexForFlow(s.flowEff + flow);
                if (req >= 0 && Rules.DN[req] > s.dn) {
                    cost += (firstPartial ? partial : s.length) * Rules.COST_RECON[req];
                }
                firstPartial = false;
            }
            return cost;
        }

        /** Добавляет ОКС в копию дерева по кандидату; null, если присоединение невозможно. */
        Tree attach(Tree tree, Cand c, double[][] pathFromTarget, PlanModel.Target target,
                    PlanModel.ConnectionPoint cp, Exempt ex) {
            Tree t = tree.copy();
            double[][] pts = trimAtOwn(Geo.reverse(pathFromTarget), ex.own);
            Tree.Node from;
            switch (c.kind) {
                case Cand.SEGMENT:
                    from = t.addTieSegment(c.segment, c.along, c.xy);
                    break;
                case Cand.CHAMBER:
                    from = t.addTieChamber(c.chamber);
                    break;
                case Cand.NODE:
                    from = null;
                    for (Tree.Node n : t.nodes) {
                        if (n.id == c.nodeId) {
                            from = n;
                        }
                    }
                    break;
                default:
                    Tree.Edge e = t.edges.get(c.edgeIndex);
                    from = t.split(e, c.along, model);
                    break;
            }
            if (from == null || !from.canTakeBranch()) {
                return null;
            }
            t.connect(from, t.addTerminal(target, cp, pts[pts.length - 1]), pts, model);
            return t;
        }

        /**
         * Труба не прокладывается под зданием: путь (от врезки к ОКС) обрезается там, где он впервые касается контура
         * здания самого ОКС. Терминал оказывается у стены. Если путь начинается уже внутри контура, он не меняется.
         */
        double[][] trimAtOwn(double[][] pts, List<PlanModel.Shape> own) {
            if (own.isEmpty()) {
                return pts;
            }
            for (PlanModel.Shape s : own) {
                if (Geo.inside(pts[0][0], pts[0][1], s.rings)) {
                    return pts;
                }
            }
            for (int i = 1; i < pts.length; i++) {
                double best = 2;
                for (PlanModel.Shape s : own) {
                    for (double[][] ring : s.rings) {
                        for (int j = 1; j < ring.length; j++) {
                            double[] hit = Geo.intersect(pts[i - 1], pts[i], ring[j - 1], ring[j]);
                            if (hit != null && hit[0] < best) {
                                best = hit[0];
                            }
                        }
                    }
                }
                if (best <= 1) {
                    double[] cut = {pts[i - 1][0] + best * (pts[i][0] - pts[i - 1][0]),
                            pts[i - 1][1] + best * (pts[i][1] - pts[i - 1][1])};
                    double[][] out = new double[i + 1][];
                    System.arraycopy(pts, 0, out, 0, i);
                    out[i] = cut;
                    // Совсем короткий остаток (труба упирается в стену сразу) оставляем как есть, чтобы не получить пустую трубу.
                    return Geo.length(out) < 0.05 ? pts : out;
                }
            }
            return pts;
        }

        /** Точная проверка пути: зазоры до запретных зон и отсутствие пересечений с уже проложенными новыми трубами. */
        boolean valid(double[][] pts, Tree tree, PlanModel.ConnectionPoint cp, Exempt ex) {
            double[] goal = pts[pts.length - 1];
            for (int i = 1; i < pts.length; i++) {
                if (violatesClearance(pts[i - 1], pts[i], cp.xy, ex)) {
                    return false;
                }
                for (Tree.Edge e : tree.edges) {
                    for (int j = 1; j < e.pts.length; j++) {
                        double[] hit = Geo.intersect(pts[i - 1], pts[i], e.pts[j - 1], e.pts[j]);
                        if (hit != null) {
                            double[] at = {pts[i - 1][0] + hit[0] * (pts[i][0] - pts[i - 1][0]),
                                    pts[i - 1][1] + hit[0] * (pts[i][1] - pts[i - 1][1])};
                            if (Geo.dist(at, goal) > 0.05) {
                                return false;
                            }
                        }
                    }
                }
            }
            return true;
        }

        /** Отрезок ab нарушает зазор до запретной зоны (часть в круге exemptR вокруг center не проверяется). */
        boolean violatesClearance(double[] a, double[] b, double[] center, Exempt ex) {
            double exemptR = ex.radius;
            double[] from = a;
            double len = Geo.dist(a, b);
            if (exemptR > 0 && Geo.dist(a, center) < exemptR) {
                if (Geo.dist(b, center) <= exemptR || len == 0) {
                    return false;
                }
                // Сдвигаем начало отрезка за пределы круга.
                double lo = 0;
                double hi = 1;
                for (int k = 0; k < 30; k++) {
                    double mid = (lo + hi) / 2;
                    double[] p = {a[0] + mid * (b[0] - a[0]), a[1] + mid * (b[1] - a[1])};
                    if (Geo.dist(p, center) < exemptR) {
                        lo = mid;
                    } else {
                        hi = mid;
                    }
                }
                from = new double[]{a[0] + hi * (b[0] - a[0]), a[1] + hi * (b[1] - a[1])};
            }
            double minX = Math.min(from[0], b[0]);
            double maxX = Math.max(from[0], b[0]);
            double minY = Math.min(from[1], b[1]);
            double maxY = Math.max(from[1], b[1]);
            for (PlanModel.Shape s : model.shapes) {
                if (!s.rule.forbidden || ex.own.contains(s)) {
                    continue;
                }
                double d = s.rule.minDistance;
                if (s.maxX + d < minX || s.minX - d > maxX || s.maxY + d < minY || s.minY - d > maxY) {
                    continue;
                }
                if (s.area && (Geo.inside(from[0], from[1], s.rings) || Geo.inside(b[0], b[1], s.rings))) {
                    return true;
                }
                double best = Double.MAX_VALUE;
                for (double[][] ring : s.rings) {
                    for (int i = 1; i < ring.length; i++) {
                        best = Math.min(best, Geo.segmentSegment(from, b, ring[i - 1], ring[i]));
                    }
                }
                if (best < d - 1e-6) {
                    return true;
                }
            }
            return false;
        }
    }
}
