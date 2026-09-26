package ru.heatplanner.plan;

import org.junit.jupiter.api.Test;
import ru.heatplanner.Utm;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Сквозные проверки на небольших синтетических наборах: разбор файла, расчёт вариантов, независимый валидатор и выгрузка.
 * Наборы задаются в метрах от опорной точки (UTM 37N) и переводятся в WGS 84, как настоящий файл.
 */
class PipelineTest {

    private static final double X0 = 414000;
    private static final double Y0 = 6173000;

    /** Сборщик GeoJSON из объектов в локальных метрах. */
    private static final class Scenario {
        private final List<String> features = new ArrayList<>();

        private static String q(Object id) {
            return id instanceof String ? "\"" + id + "\"" : String.valueOf(id);
        }

        private static String pt(double x, double y) {
            double[] ll = Utm.inverse(X0 + x, Y0 + y);
            return "[" + ll[0] + "," + ll[1] + "]";
        }

        private static String ring(double[][] pts) {
            StringBuilder sb = new StringBuilder("[");
            for (int i = 0; i < pts.length; i++) {
                sb.append(i > 0 ? "," : "").append(pt(pts[i][0], pts[i][1]));
            }
            return sb.append("]").toString();
        }

        Scenario point(Object id, String type, double x, double y, String extra) {
            features.add("{\"type\":\"Feature\",\"properties\":{\"id\":" + q(id) + ",\"object_type\":\"" + type + "\"" + extra
                    + "},\"geometry\":{\"type\":\"Point\",\"coordinates\":" + pt(x, y) + "}}");
            return this;
        }

        Scenario network(Object id, int dn, double[][] pts) {
            features.add("{\"type\":\"Feature\",\"properties\":{\"id\":" + q(id) + ",\"object_type\":\"heat_network\",\"diameter\":" + dn
                    + "},\"geometry\":{\"type\":\"LineString\",\"coordinates\":" + ring(pts) + "}}");
            return this;
        }

        Scenario polygon(Object id, String restriction, double x1, double y1, double x2, double y2) {
            double[][] pts = {{x1, y1}, {x2, y1}, {x2, y2}, {x1, y2}, {x1, y1}};
            features.add("{\"type\":\"Feature\",\"properties\":{\"id\":" + q(id) + ",\"object_type\":\"restriction\",\"restriction_type\":\""
                    + restriction + "\"},\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[" + ring(pts) + "]}}");
            return this;
        }

        Scenario line(Object id, String restriction, double[][] pts) {
            features.add("{\"type\":\"Feature\",\"properties\":{\"id\":" + q(id) + ",\"object_type\":\"restriction\",\"restriction_type\":\""
                    + restriction + "\"},\"geometry\":{\"type\":\"LineString\",\"coordinates\":" + ring(pts) + "}}");
            return this;
        }

        String json() {
            return "{\"type\":\"FeatureCollection\",\"features\":[" + String.join(",", features) + "]}";
        }
    }

    private static final class Outcome {
        PlanInput in;
        List<Variants.Variant> variants;
    }

    private static Outcome run(Scenario s) throws Exception {
        Path tmp = Files.createTempFile("scenario-", ".geojson");
        try {
            Files.write(tmp, s.json().getBytes(StandardCharsets.UTF_8));
            RuleSet rules = RuleSet.strict();
            Outcome o = new Outcome();
            o.in = PlanInput.read(tmp, null, rules, false);
            ObstacleSet set = new ObstacleSet(o.in.obstacles, rules);
            o.variants = Variants.generate(o.in, set, new JointPlanner(o.in, set, rules), 3, false);
            return o;
        } finally {
            Files.deleteIfExists(tmp);
        }
    }

    private static void assertClean(Variants.Variant v) {
        for (PlanValidator.Violation x : v.violations) {
            assertFalse(x.group == PlanValidator.Group.RULE, "нарушение правил: " + x.message);
        }
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> summary(Map<String, Object> export, String variantId) {
        for (Object f : (List<Object>) export.get("features")) {
            Map<String, Object> p = (Map<String, Object>) ((Map<String, Object>) f).get("properties");
            if ("variant_summary".equals(p.get("object_type")) && variantId.equals(p.get("variant_id"))) {
                return p;
            }
        }
        return null;
    }

    /** Пример из раздела 7.3 приложения: 100 м, ДУ 100, 20 т/ч, врезка в существующую камеру = 13 974 800 ₽, S = 0,6913. */
    @Test
    void specificationExampleGivesTheDocumentedCostAndScore() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -8, 0, "")
                .network("net", 100, new double[][]{{-8, 0}, {0, 0}})
                .point("input_chamber_1", "heat_chamber", 0, 0, "")
                .point("input_oks_1", "oks_connection_point", 0, 100, ",\"flow_tph\":20");
        Outcome o = run(s);
        assertFalse(o.variants.isEmpty());
        Variants.Variant best = o.variants.get(0);
        assertClean(best);
        assertEquals(1, best.solution.branches.size());
        Map<String, Object> summary = summary(OutputBuilder.build(o.variants), "v1");
        assertNotNull(summary);
        double cost = (Double) summary.get("calculated_cost");
        double length = (Double) summary.get("new_network_length");
        // Длина по проекции UTM отличается от плоской на доли процента, поэтому допуск 0,5 %.
        assertEquals(100.0, length, 0.5);
        assertEquals(13_974_800, cost, 13_974_800 * 0.005);
        assertEquals(0.6913, (Double) summary.get("score"), 0.005);
        assertEquals(1, summary.get("existing_chamber_tie_in_count"));
        assertEquals(5_000_000.0, (Double) summary.get("existing_chamber_tie_in_cost"), 1e-6);
    }

    /** Дорога-полигон на пути ко всем точкам: пересечение только специальным проходом (K = 1,60, отдельный участок). */
    @Test
    void roadPolygonIsCrossedBySpecialPass() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -100, 0, "")
                .network("net", 300, new double[][]{{-100, 0}, {100, 0}})
                .point("ch", "heat_chamber", 0, 0, "")
                .polygon("road", "road", -400, 90, 400, 110)
                .point(1, "oks_connection_point", -30, 200, ",\"flow_tph\":10")
                .point(2, "oks_connection_point", 40, 210, ",\"flow_tph\":12");
        Outcome o = run(s);
        assertFalse(o.variants.isEmpty());
        Variants.Variant best = o.variants.get(0);
        assertClean(best);
        assertEquals(2, best.solution.branches.size(), "обе точки подключены");
        boolean special = false;
        for (Branch b : best.solution.branches) {
            for (Route.Leg leg : b.route.legs) {
                special |= leg.special && leg.kSpec == 1.60;
            }
        }
        assertTrue(special, "переход дороги должен быть специальным проходом с коэффициентом 1,60");
    }

    /** Газопровод, парк, вода и здание перекрывают часть пути: маршрут идёт в обход с отступами, нарушений нет. */
    @Test
    void forbiddenAndLinearObjectsAreAvoidedWithClearances() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -150, 0, "")
                .network("net", 200, new double[][]{{-150, 0}, {150, 0}})
                .point("ch", "heat_chamber", 0, 0, "")
                .polygon("park", "park", -60, 60, 60, 90)
                .polygon("water", "water", 80, 40, 140, 130)
                .polygon("house", "oks", -140, 40, -90, 120)
                .line("gas", "gas_pipeline", new double[][]{{-70, 140}, {70, 140}})
                .point(1, "oks_connection_point", 0, 200, ",\"flow_tph\":15")
                .point(2, "oks_connection_point", -80, 190, ",\"flow_tph\":8");
        Outcome o = run(s);
        assertFalse(o.variants.isEmpty());
        Variants.Variant best = o.variants.get(0);
        assertClean(best);
        assertEquals(2, best.solution.branches.size());
        assertTrue(best.solution.evaluation.length > 200, "обход длиннее прямого расстояния");
    }

    /** Точка во внутреннем дворе здания: подойти к ней по правилу нельзя, точка остаётся неподключённой с причиной и штрафом. */
    @Test
    void pointInsideAClosedCourtyardIsReportedAsUnconnected() throws Exception {
        String courtyard = "{\"type\":\"Feature\",\"properties\":{\"id\":\"b\",\"object_type\":\"restriction\",\"restriction_type\":\"oks\"},"
                + "\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[" + Scenario.ring(new double[][]{{-40, 100}, {40, 100}, {40, 180}, {-40, 180}, {-40, 100}})
                + "," + Scenario.ring(new double[][]{{-8, 130}, {8, 130}, {8, 150}, {-8, 150}, {-8, 130}}) + "]}}";
        Scenario s = new Scenario()
                .point("src", "source", -100, 0, "")
                .network("net", 200, new double[][]{{-100, 0}, {100, 0}})
                .point("ch", "heat_chamber", 0, 0, "")
                .point(1, "oks_connection_point", 0, 140, ",\"flow_tph\":10")
                .point(2, "oks_connection_point", 120, 60, ",\"flow_tph\":10");
        s.features.add(courtyard);
        Outcome o = run(s);
        Variants.Variant best = o.variants.get(0);
        assertClean(best);
        assertEquals(1, best.solution.unconnected.size());
        assertEquals("1", String.valueOf(best.solution.unconnected.get(0).target.id.value()));
        assertFalse(best.solution.unconnected.get(0).reason.isEmpty());
        Map<String, Object> summary = summary(OutputBuilder.build(o.variants), "v1");
        assertEquals(Rules.penalty(10), (Double) summary.get("unconnected_penalty"), 1e-6);
    }

    /** Выгрузка сохраняет тип идентификатора точки: число остаётся числом, строка — строкой. */
    @Test
    void identifierTypesArePreservedInTheExport() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -8, 0, "")
                .network("net", 100, new double[][]{{-8, 0}, {0, 0}})
                .point(7, "heat_chamber", 0, 0, "")
                .point("cp-a", "oks_connection_point", 0, 60, ",\"flow_tph\":5")
                .point(42, "oks_connection_point", 30, 70, ",\"flow_tph\":5");
        Outcome o = run(s);
        Map<String, Object> export = OutputBuilder.build(o.variants);
        boolean numeric = false;
        boolean text = false;
        @SuppressWarnings("unchecked")
        List<Object> features = (List<Object>) export.get("features");
        for (Object f : features) {
            @SuppressWarnings("unchecked")
            Map<String, Object> p = (Map<String, Object>) ((Map<String, Object>) f).get("properties");
            if ("heat_network".equals(p.get("object_type"))) {
                numeric |= Long.valueOf(42).equals(p.get("start_node_id"));
                text |= "cp-a".equals(p.get("start_node_id"));
            }
        }
        assertTrue(numeric && text, "идентификаторы 42 (число) и cp-a (строка) должны сохранить тип");
    }
    /** Трамвайные пути пересекаются специальным проходом с коэффициентом 1,75. */
    @Test
    void tramTracksAreCrossedWithTheirOwnCoefficient() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -100, 0, "")
                .network("net", 300, new double[][]{{-100, 0}, {100, 0}})
                .point("ch", "heat_chamber", 0, 0, "")
                .polygon("tram", "tram_tracks", -400, 90, 400, 100)
                .point(1, "oks_connection_point", 10, 200, ",\"flow_tph\":10");
        Outcome o = run(s);
        Variants.Variant best = o.variants.get(0);
        assertClean(best);
        assertEquals(1, best.solution.branches.size());
        boolean tram = false;
        for (Route.Leg leg : best.solution.branches.get(0).route.legs) {
            tram |= leg.special && leg.kSpec == 1.75;
        }
        assertTrue(tram);
    }

    /** Железная дорога, социальная и запрещённая территории, кабель: обход с отступами, нарушений нет, программа не падает. */
    @Test
    void otherRestrictionTypesAreRespected() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -200, 0, "")
                .network("net", 200, new double[][]{{-200, 0}, {200, 0}})
                .point("ch", "heat_chamber", 0, 0, "")
                .polygon("rail", "railway", -50, 60, 50, 66)
                .polygon("social", "social_area", 60, 50, 120, 110)
                .polygon("forbidden", "prohibited_site", -130, 50, -70, 110)
                .line("cable", "power_cable", new double[][]{{-60, 150}, {60, 150}})
                .point(1, "oks_connection_point", 0, 220, ",\"flow_tph\":9")
                .point(2, "oks_connection_point", 90, 200, ",\"flow_tph\":6");
        Outcome o = run(s);
        assertFalse(o.variants.isEmpty());
        assertClean(o.variants.get(0));
        assertEquals(2, o.variants.get(0).solution.branches.size());
    }

    /** У существующей камеры уже четыре участка: врезка в неё невозможна, подключение идёт в новую камеру. */
    @Test
    void fullExistingChamberIsNotUsed() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -100, 0, "")
                .network("a", 200, new double[][]{{-100, 0}, {0, 0}})
                .network("b", 200, new double[][]{{0, 0}, {100, 0}})
                .network("c", 150, new double[][]{{0, 0}, {0, -50}})
                .network("d", 150, new double[][]{{0, 0}, {0, 50}})
                .point("full", "heat_chamber", 0, 0, "")
                .point(1, "oks_connection_point", 3, 120, ",\"flow_tph\":10");
        Outcome o = run(s);
        Variants.Variant best = o.variants.get(0);
        assertClean(best);
        assertEquals(1, best.solution.branches.size());
        assertEquals(0, best.solution.evaluation.existingTieIns, "в камеру с четырьмя участками врезаться нельзя");
    }

    /** Нет ни одной точки подключения: расчёт не падает, вариант пустой. */
    @Test
    void noTargetsDoesNotFail() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -100, 0, "")
                .network("net", 100, new double[][]{{-100, 0}, {100, 0}});
        Outcome o = run(s);
        for (Variants.Variant v : o.variants) {
            assertEquals(0, v.solution.branches.size());
        }
        Map<String, Object> export = OutputBuilder.build(o.variants);
        assertNotNull(export.get("features"));
    }

    /** Расход больше пропускной способности самого большого ДУ: точка неподключена, причина названа. */
    @Test
    void flowAboveTheLargestDiameterIsReported() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -100, 0, "")
                .network("net", 100, new double[][]{{-100, 0}, {100, 0}})
                .point("ch", "heat_chamber", 0, 0, "")
                .point(1, "oks_connection_point", 0, 80, ",\"flow_tph\":99999");
        Outcome o = run(s);
        Variants.Variant best = o.variants.get(0);
        assertEquals(1, best.solution.unconnected.size());
        assertTrue(best.solution.unconnected.get(0).reason.contains("пропускной способности"));
    }

    /** Две точки в одном здании: у обеих собственный финальный участок, отступ до своего здания на нём не проверяется. */
    @Test
    void twoPointsInsideOneBuildingBothConnect() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -100, 0, "")
                .network("net", 200, new double[][]{{-100, 0}, {100, 0}})
                .point("ch", "heat_chamber", 0, 0, "")
                .polygon("house", "oks", -30, 60, 30, 100)
                .point(1, "oks_connection_point", -15, 75, ",\"flow_tph\":5")
                .point(2, "oks_connection_point", 15, 85, ",\"flow_tph\":5");
        Outcome o = run(s);
        Variants.Variant best = o.variants.get(0);
        assertClean(best);
        assertEquals(2, best.solution.branches.size());
        for (Branch b : best.solution.branches) {
            assertTrue(b.route.legs.get(0).terminal, "первый участок — финальный прямой к точке");
        }
    }

    /** Точка подключения далеко от сети (сотни метров) и препятствий нет: обычная прямая трасса. */
    @Test
    void distantTargetWithoutObstaclesGetsAStraightRoute() throws Exception {
        Scenario s = new Scenario()
                .point("src", "source", -100, 0, "")
                .network("net", 300, new double[][]{{-100, 0}, {100, 0}})
                .point("ch", "heat_chamber", 0, 0, "")
                .point(1, "oks_connection_point", 0, 380, ",\"flow_tph\":30");
        Outcome o = run(s);
        Variants.Variant best = o.variants.get(0);
        assertClean(best);
        assertEquals(1, best.solution.branches.size());
        assertEquals(1, best.solution.branches.get(0).route.legs.size(), "без препятствий трасса — один прямой участок");
    }
    /**
     * Точка ровно на границе здания, заданная в градусах: в метрах граница слегка искривляется, и точка оказывается снаружи
     * на миллиметры. Она всё равно принадлежит зданию и не должна считаться лежащей в чужой зоне отступа.
     */
    @Test
    void pointOnTheBuildingBoundaryGivenInDegreesBelongsToTheBuilding() throws Exception {
        String json = "{\"type\":\"FeatureCollection\",\"features\":["
                + "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[37.6,55.75]},\"properties\":{\"id\":\"s\",\"object_type\":\"source\"}},"
                + "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[[37.6,55.75],[37.602,55.751]]},\"properties\":{\"id\":\"n\",\"object_type\":\"heat_network\",\"diameter\":300}},"
                + "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[37.602,55.751]},\"properties\":{\"id\":\"c\",\"object_type\":\"heat_chamber\"}},"
                + "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[37.605,55.7532]},\"properties\":{\"id\":\"cp\",\"object_type\":\"oks_connection_point\",\"flow_tph\":80.0}},"
                + "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[37.605,55.753],[37.6055,55.753],[37.6055,55.7534],[37.605,55.7534],[37.605,55.753]]]},"
                + "\"properties\":{\"id\":\"b\",\"object_type\":\"restriction\",\"restriction_type\":\"oks\"}}]}";
        Path tmp = Files.createTempFile("boundary-", ".geojson");
        try {
            Files.write(tmp, json.getBytes(StandardCharsets.UTF_8));
            RuleSet rules = RuleSet.strict();
            PlanInput in = PlanInput.read(tmp, null, rules, false);
            ObstacleSet set = new ObstacleSet(in.obstacles, rules);
            List<Variants.Variant> variants = Variants.generate(in, set, new JointPlanner(in, set, rules), 3, false);
            Variants.Variant best = variants.get(0);
            assertClean(best);
            assertEquals(1, best.solution.branches.size(), best.solution.unconnected.isEmpty() ? "" : best.solution.unconnected.get(0).reason);
        } finally {
            Files.deleteIfExists(tmp);
        }
    }
}