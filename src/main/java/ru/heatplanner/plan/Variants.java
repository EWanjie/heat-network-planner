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

        Variant(JointPlanner.Solution solution, String strategy, List<PlanValidator.Violation> violations) {
            this.solution = solution;
            this.strategy = strategy;
            this.violations = violations;
        }
    }

    /** Доля ветвей с другой врезкой, начиная с которой варианты считаются разными. */
    static final double MIN_DIFFERENCE = 0.25;

    private Variants() {
    }

    public static List<Variant> generate(PlanInput in, ObstacleSet exact, JointPlanner planner, int maxVariants) {
        List<PlanInput.Target> nearest = new ArrayList<>(in.targets);
        nearest.sort(Comparator.comparingDouble(t -> distanceToNetwork(in, t)));
        List<PlanInput.Target> farthest = new ArrayList<>(nearest);
        java.util.Collections.reverse(farthest);
        List<PlanInput.Target> byFlow = new ArrayList<>(in.targets);
        byFlow.sort(Comparator.comparingDouble((PlanInput.Target t) -> -t.flow));

        List<Variant> all = new ArrayList<>();
        add(all, in, exact, planner, nearest, Router.Profile.BALANCED, "Сначала ближние к сети точки, баланс стоимости и длины");
        add(all, in, exact, planner, farthest, Router.Profile.BALANCED, "Сначала дальние от сети точки, баланс стоимости и длины");
        add(all, in, exact, planner, nearest, Router.Profile.COMPACT, "Сначала ближние точки, наименьшая длина новой сети");
        add(all, in, exact, planner, nearest, Router.Profile.ECONOMIC, "Сначала ближние точки, наименьшая стоимость труб");
        add(all, in, exact, planner, byFlow, Router.Profile.BALANCED, "Сначала точки с наибольшим расходом");

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
        return chosen;
    }

    private static void add(List<Variant> all, PlanInput in, ObstacleSet exact, JointPlanner planner,
                            List<PlanInput.Target> order, Router.Profile profile, String strategy) {
        JointPlanner.Solution s = planner.planBest(order, 3, profile);
        List<PlanValidator.Violation> v = PlanValidator.validateTree(in, exact, s.branches, s.evaluation);
        for (PlanValidator.Violation x : v) {
            if (x.group == PlanValidator.Group.RULE) {
                return;
            }
        }
        all.add(new Variant(s, strategy, v));
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