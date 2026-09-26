package ru.heatplanner.plan;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Несколько существенно разных вариантов сети. Варианты строятся разными стратегиями (порядок добавления целей и
 * показатель, по которому ищется трасса), затем оставляются те, что различаются структурой сети, а не смещением
 * линий: не менее {@link #MIN_DIFFERENCE} ветвей должны иметь другую врезку. Все варианты проверяются валидатором,
 * с нарушениями правил в результат не попадают.
 */
public final class Variants {

    /** Один вариант с описанием стратегии. */
    public static final class Variant {
        public final JointPlanner.Solution solution;
        public final String strategy;
        public final List<PlanValidator.Violation> violations;
        /** Допущения, на которых построен вариант (пусто — вариант строго по данным и правилам). */
        public final List<String> assumptions;
        final Router.Profile profile;
        /** Валидатор нашёл нарушение правил: вариант показывается только если правильных нет. */
        public final boolean invalid;
        /** Тот же вариант до перестройки цепочек (null, если перестройка ничего не изменила). */
        public Variant previous;
        /** Номер стратегии (порядок добавления целей и показатель): по нему дополнительный вариант ставится к решению. */
        public int key = -1;

        Variant(JointPlanner.Solution solution, String strategy, List<PlanValidator.Violation> violations) {
            this(solution, strategy, violations, Router.Profile.BALANCED);
        }

        Variant(JointPlanner.Solution solution, String strategy, List<PlanValidator.Violation> violations, Router.Profile profile) {
            this.solution = solution;
            this.strategy = strategy;
            this.violations = violations;
            this.assumptions = assumptionsOf(solution);
            this.profile = profile;
            boolean bad = false;
            for (PlanValidator.Violation x : violations) {
                bad |= x.group == PlanValidator.Group.RULE;
            }
            this.invalid = bad;
        }
    }

    /** Доля ветвей с другой врезкой, начиная с которой варианты считаются разными. */
    static final double MIN_DIFFERENCE = 0.25;

    private Variants() {
    }

    /** Допущения решения: запасной подход к зданию и проход через дорогу с шириной, принятой по классу. */
    static List<String> assumptionsOf(JointPlanner.Solution s) {
        boolean fallback = false;
        boolean road = false;
        for (Branch b : s.branches) {
            fallback |= !b.route.start.nearestBoundary;
            for (Route.Leg leg : b.route.legs) {
                for (Obstacle o : leg.crossed) {
                    road |= o.assumedWidth;
                }
            }
        }
        List<String> out = new ArrayList<>();
        if (road) {
            out.add("ширина дороги принята по её классу в OpenStreetMap");
        }
        if (fallback) {
            out.add("подход к зданию с ближайшей допустимой стороны, а не с ближайшей границы");
        }
        return out;
    }

    /**
     * Все стратегии без отбора и перестройки цепочек. Стратегии считаются одновременно: у них общие графы видимости,
     * а поиск внутри каждой тоже параллельный. Номер стратегии — в {@link Variant#key}.
     */
    public static List<Variant> buildAll(PlanInput in, ObstacleSet exact, JointPlanner planner) {
        List<PlanInput.Target> nearest = new ArrayList<>(in.targets);
        nearest.sort(Comparator.comparingDouble(t -> distanceToNetwork(in, t)));
        List<PlanInput.Target> farthest = new ArrayList<>(nearest);
        java.util.Collections.reverse(farthest);
        List<PlanInput.Target> byFlow = new ArrayList<>(in.targets);
        byFlow.sort(Comparator.comparingDouble((PlanInput.Target t) -> -t.flow));

        List<Object[]> configs = new ArrayList<>();
        configs.add(new Object[]{nearest, Router.Profile.BALANCED, "Сначала ближние к сети точки, баланс стоимости и длины"});
        configs.add(new Object[]{farthest, Router.Profile.BALANCED, "Сначала дальние от сети точки, баланс стоимости и длины"});
        configs.add(new Object[]{nearest, Router.Profile.COMPACT, "Сначала ближние точки, наименьшая длина новой сети"});
        configs.add(new Object[]{byFlow, Router.Profile.BALANCED, "Сначала точки с наибольшим расходом"});
        java.util.concurrent.ExecutorService pool = java.util.concurrent.Executors.newFixedThreadPool(configs.size());
        List<java.util.concurrent.Future<Variant>> futures = new ArrayList<>();
        for (int k = 0; k < configs.size(); k++) {
            Object[] c = configs.get(k);
            int key = k;
            @SuppressWarnings("unchecked")
            List<PlanInput.Target> order = (List<PlanInput.Target>) c[0];
            futures.add(pool.submit(() -> {
                Variant v = build(in, exact, planner, order, (Router.Profile) c[1], (String) c[2]);
                v.key = key;
                return v;
            }));
        }
        List<Variant> all = new ArrayList<>();
        try {
            for (java.util.concurrent.Future<Variant> f : futures) {
                Variant v = f.get();
                if (v != null) {
                    all.add(v);
                }
            }
        } catch (InterruptedException | java.util.concurrent.ExecutionException e) {
            throw new IllegalStateException(e);
        } finally {
            pool.shutdown();
        }
        return all;
    }

    /** Перестройка цепочек одного варианта; если она не улучшает результат или ломает правила — вариант как был. */
    public static Variant polish(PlanInput in, ObstacleSet exact, JointPlanner planner, Variant c) {
        Router.Profile profile = profileOf(c);
        JointPlanner.Solution s = planner.improve(planner.rebuildChains(planner.improve(c.solution, profile, 2), profile), profile, 2);
        if (s.score() >= c.solution.score() - 1e-9) {
            return c;
        }
        List<PlanValidator.Violation> v = PlanValidator.validateTree(in, exact, s.branches, s.evaluation);
        for (PlanValidator.Violation x : v) {
            if (x.group == PlanValidator.Group.RULE) {
                return c;
            }
        }
        Variant improved = new Variant(s, c.strategy, v, c.profile);
        improved.previous = c;
        improved.key = c.key;
        return improved;
    }

    /** Основные варианты: непохожие друг на друга, лучшие по показателю S. */
    public static List<Variant> generate(PlanInput in, ObstacleSet exact, JointPlanner planner, int maxVariants,
                                         boolean unused) {
        List<Variant> all = buildAll(in, exact, planner);
        // Варианты с нарушением правил не показываются, если есть правильные; иначе лучший из них показывается с пометкой.
        List<Variant> valid = new ArrayList<>();
        for (Variant v : all) {
            if (!v.invalid) {
                valid.add(v);
            }
        }
        if (!valid.isEmpty()) {
            all = valid;
        }
        all.sort(Comparator.comparingDouble(v -> v.solution.score()));
        List<Variant> chosen = new ArrayList<>();
        for (Variant v : all) {
            boolean distinct = true;
            for (Variant c : chosen) {
                if (difference(v.solution, c.solution) < MIN_DIFFERENCE) {
                    distinct = false;
                    break;
                }
            }
            if (distinct) {
                chosen.add(v);
            }
            if (chosen.size() >= maxVariants) {
                break;
            }
        }
        // Дорогая перестройка цепочек — только для отобранных вариантов: по всем стратегиям она заняла бы слишком много.
        List<java.util.concurrent.Future<Variant>> polished = new ArrayList<>();
        java.util.concurrent.ExecutorService polishPool = java.util.concurrent.Executors.newFixedThreadPool(Math.max(1, chosen.size()));
        try {
            for (Variant c : chosen) {
                polished.add(polishPool.submit(() -> polish(in, exact, planner, c)));
            }
            List<Variant> result = new ArrayList<>();
            for (java.util.concurrent.Future<Variant> f : polished) {
                result.add(f.get());
            }
            result.sort(Comparator.comparingDouble(x -> x.solution.score()));
            return result;
        } catch (InterruptedException | java.util.concurrent.ExecutionException e) {
            throw new IllegalStateException(e);
        } finally {
            polishPool.shutdown();
        }
    }

    private static Router.Profile profileOf(Variant v) {
        return v.profile;
    }

    private static Variant build(PlanInput in, ObstacleSet exact, JointPlanner planner, List<PlanInput.Target> order,
                                 Router.Profile profile, String strategy) {
        JointPlanner.Solution s = planner.planBest(order, 3, profile);
        List<PlanValidator.Violation> v = PlanValidator.validateTree(in, exact, s.branches, s.evaluation);
        return new Variant(s, strategy, v, profile);
    }
    static double distanceToNetwork(PlanInput in, PlanInput.Target t) {
        double d = Double.MAX_VALUE;
        org.locationtech.jts.geom.Point p = new org.locationtech.jts.geom.GeometryFactory().createPoint(t.xy);
        for (PlanInput.Segment s : in.segments) {
            d = Math.min(d, s.line.distance(p));
        }
        return d;
    }

    /** Подпись врезки ветви: к какой ветви она примыкает или в какое место сети (с точностью до 10 м). */
    private static Map<String, String> signatures(JointPlanner.Solution s) {
        Map<String, String> out = new HashMap<>();
        for (Branch b : s.branches) {
            String sig;
            if (b.parent >= 0) {
                sig = "branch:" + s.branches.get(b.parent).target.id.value();
            } else {
                sig = "net:" + Math.round(b.attach.x / 10) + ":" + Math.round(b.attach.y / 10);
            }
            out.put(String.valueOf(b.target.id.value()), sig);
        }
        return out;
    }

    /** Доля целей, у которых врезка разная (или цель подключена только в одном из вариантов). */
    static double difference(JointPlanner.Solution a, JointPlanner.Solution b) {
        Map<String, String> sa = signatures(a);
        Map<String, String> sb = signatures(b);
        java.util.Set<String> keys = new java.util.HashSet<>(sa.keySet());
        keys.addAll(sb.keySet());
        if (keys.isEmpty()) {
            return 0;
        }
        int diff = 0;
        for (String k : keys) {
            if (!java.util.Objects.equals(sa.get(k), sb.get(k))) {
                diff++;
            }
        }
        return (double) diff / keys.size();
    }
}