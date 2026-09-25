package ru.heatplanner.plan;

import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Выходной GeoJSON: ссылки на узлы, совпадение концов участков с узлами, согласованность итогов. */
class OutputTest {

    private static final GeometryFactory F = new GeometryFactory();
    private static final ObstacleSet NONE = new ObstacleSet(List.of(), RuleSet.strict());

    private static Coordinate c(double x, double y) {
        return new Coordinate(x, y);
    }

    private static Goals.Attach network() {
        PlanInput.Segment s = new PlanInput.Segment(Id.text("net"), 300, F.createLineString(new Coordinate[]{c(-200, 0), c(200, 0)}));
        return new Goals.Attach(Goals.Kind.NEW_CHAMBER, null, s, 300);
    }

    private static Branch branch(Id id, double flow, int parent, double parentPos, boolean[] special, Coordinate... pts) {
        List<Route.Leg> legs = new ArrayList<>();
        for (int i = 1; i < pts.length; i++) {
            boolean sp = special != null && special[i - 1];
            legs.add(new Route.Leg(pts[i - 1], pts[i], sp, sp ? 1.6 : 1, List.of(), false, false));
        }
        PlanInput.Target target = new PlanInput.Target(id, pts[0], flow);
        Start start = new Start(pts[0], null, Collections.emptySet(), null, target);
        Coordinate end = pts[pts.length - 1];
        Goals.Attach existing = parent < 0 ? network() : null;
        Goal goal = new Goal(end, Collections.emptySet(), null, existing);
        return new Branch(target, new Route(legs, start, goal, 100), parent, existing, end, parentPos);
    }

    private static Map<String, Object> props(Object feature) {
        return (Map<String, Object>) ((Map<String, Object>) feature).get("properties");
    }

    private static Map<String, Object> build(List<Branch> bs) {
        TreeEvaluator.Result ev = TreeEvaluator.evaluate(null, NONE, bs);
        assertTrue(ev.feasible, ev.reason);
        JointPlanner.Solution s = new JointPlanner.Solution(bs, new ArrayList<>(), ev);
        return OutputBuilder.build(List.of(new Variants.Variant(s, "тест", new ArrayList<>())));
    }

    @SuppressWarnings("unchecked")
    private static List<Object> features(Map<String, Object> fc) {
        return (List<Object>) fc.get("features");
    }

    @Test
    void everyNodeReferenceResolvesAndEndsMatchNodes() {
        Branch a = branch(Id.number(101), 20, -1, 0, null, c(50, 40), c(50, 0));
        Branch b = branch(Id.text("cp-b"), 20, 0, 20, null, c(80, 40), c(80, 20), c(50, 20));
        Map<String, Object> fc = build(List.of(a, b));
        Set<Object> nodes = new HashSet<>();
        Map<Object, List<Double>> nodeXy = new HashMap<>();
        nodes.add(101L);
        nodes.add("cp-b");
        int networks = 0;
        int chambers = 0;
        for (Object f : features(fc)) {
            Map<String, Object> p = props(f);
            if ("heat_chamber".equals(p.get("object_type"))) {
                chambers++;
                nodes.add(p.get("id"));
            }
        }
        assertEquals(2, chambers);
        for (Object f : features(fc)) {
            Map<String, Object> p = props(f);
            if (!"heat_network".equals(p.get("object_type"))) {
                continue;
            }
            networks++;
            assertTrue(nodes.contains(p.get("start_node_id")), "начальный узел " + p.get("start_node_id"));
            assertTrue(nodes.contains(p.get("end_node_id")), "конечный узел " + p.get("end_node_id"));
            assertEquals("v1", p.get("variant_id"));
        }
        // Ветвь A режется узлом-камерой на два участка, ветвь B — один участок с поворотом внутри.
        assertEquals(3, networks);
    }

    @Test
    void idsKeepTheirTypeAndSummaryMatchesSegments() {
        Branch a = branch(Id.number(101), 20, -1, 0, null, c(50, 40), c(50, 0));
        Map<String, Object> fc = build(List.of(a));
        double sum = 0;
        double length = 0;
        Object startId = null;
        Map<String, Object> summary = null;
        for (Object f : features(fc)) {
            Map<String, Object> p = props(f);
            if ("heat_network".equals(p.get("object_type"))) {
                sum += (Double) p.get("cost");
                length += (Double) p.get("length");
                startId = p.get("start_node_id");
            } else if ("heat_chamber".equals(p.get("object_type"))) {
                sum += (Double) p.get("cost");
            } else if ("variant_summary".equals(p.get("object_type"))) {
                summary = p;
            }
        }
        assertEquals(101L, startId);
        assertEquals(sum, (Double) summary.get("construction_cost"), 0.005);
        assertEquals(length, (Double) summary.get("new_network_length"), 0.005);
        assertEquals(1, summary.get("rank"));
        assertTrue(((List<?>) summary.get("unconnected_oks_ids")).isEmpty());
    }

    @Test
    void specialPassIsSplitOffWithTechnicalNodes() {
        Branch a = branch(Id.text("cp"), 20, -1, 0, new boolean[]{false, true, false},
                c(0, 100), c(0, 60), c(0, 30), c(0, 0));
        Map<String, Object> fc = build(List.of(a));
        int nodes = 0;
        int special = 0;
        int base = 0;
        for (Object f : features(fc)) {
            Map<String, Object> p = props(f);
            if ("technical_node".equals(p.get("object_type"))) {
                nodes++;
            }
            if ("heat_network".equals(p.get("object_type"))) {
                if ("special".equals(p.get("laying_method"))) {
                    special++;
                } else {
                    base++;
                }
            }
        }
        assertEquals(2, nodes);
        assertEquals(1, special);
        assertEquals(2, base);
    }
}