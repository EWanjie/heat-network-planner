package ru.heatplanner.plan;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.Polygon;
import ru.heatplanner.Utm;

import java.io.ByteArrayInputStream;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Правила приложения и запретные зоны: значения таблиц, отступы по ДУ, проверка отрезка по всей длине. */
class GeometryRulesTest {

    private static final GeometryFactory F = new GeometryFactory();

    private static Polygon rect(double x1, double y1, double x2, double y2) {
        return F.createPolygon(new Coordinate[]{new Coordinate(x1, y1), new Coordinate(x2, y1), new Coordinate(x2, y2),
                new Coordinate(x1, y2), new Coordinate(x1, y1)});
    }

    private static Obstacle obstacle(int index, String type, Geometry g) {
        return new Obstacle(index, Id.text("o" + index), Rules.RESTRICTIONS.get(type), g, 0, "input", false, false);
    }

    private static Coordinate c(double x, double y) {
        return new Coordinate(x, y);
    }

    private static ObstacleSet set(Obstacle... obstacles) {
        return new ObstacleSet(List.of(obstacles), RuleSet.strict());
    }

    // ---- Таблицы приложения -----------------------------------------------------------------------------------------

    @Test
    void tablesMatchAppendix() {
        assertEquals(89748, Rules.PRICE[Rules.indexOfDn(100)]);
        assertEquals(0.510, Rules.pairWidth(100), 1e-12);
        assertEquals(1718, Rules.MAX_LENGTH[Rules.indexOfDn(300)]);
        assertEquals(3_000_000, Rules.chamberCost(200));
        assertEquals(5_000_000, Rules.chamberCost(250));
        assertEquals(5_000_000, Rules.chamberCost(500));
        assertEquals(8_000_000, Rules.chamberCost(600));
        assertEquals(8_000_000, Rules.chamberCost(1000));
        assertEquals(12_000_000, Rules.chamberCost(1200));
        assertEquals(100_000_000 + 500_000 * 20.0, Rules.penalty(20.0));
    }

    /** Контрольный пример приложения: 100 м ДУ 100 и одна врезка в существующую камеру. */
    @Test
    void appendixExampleCostAndScore() {
        double pipe = 100 * Rules.PRICE[Rules.indexOfDn(100)];
        assertEquals(8_974_800, pipe, 1e-6);
        double construction = pipe + Rules.EXISTING_CHAMBER_TIE_IN;
        assertEquals(13_974_800, construction, 1e-6);
        assertEquals(0.6913, Rules.score(construction, 100), 5e-5);
    }

    @Test
    void diameterByFlowTakesMinimalSufficient() {
        assertEquals(Rules.indexOfDn(100), Rules.indexForFlow(20.0));
        assertEquals(Rules.indexOfDn(50), Rules.indexForFlow(3.5));
        assertEquals(Rules.indexOfDn(65), Rules.indexForFlow(3.6));
        assertEquals(-1, Rules.indexForFlow(30_000));
    }

    @Test
    void oksClearanceDependsOnDiameter() {
        Rules.RestrictionRule oks = Rules.RESTRICTIONS.get("oks");
        assertEquals(5.0, oks.clearance(400));
        assertEquals(7.0, oks.clearance(500));
        assertEquals(7.0, oks.clearance(800));
        assertEquals(9.0, oks.clearance(900));
        assertEquals(1.0, Rules.RESTRICTIONS.get("water").clearance(1400));
    }

    @Test
    void identifiersKeepTheirType() {
        assertNotEquals(Id.text("1"), Id.number(1));
        assertEquals(Id.text("1"), Id.text("1"));
        assertEquals("1", Id.number(1).toString());
        assertEquals("\"1\"", Id.text("1").toString());
    }

    // ---- Отступы ------------------------------------------------------------------------------------------------------

    @Test
    void axisDistanceIsClearancePlusHalfEnvelopes() {
        Obstacle building = obstacle(0, "oks", rect(0, 0, 10, 10));
        assertEquals(5.0 + 0.510 / 2, building.axisDistance(100), 1e-12);
        Obstacle gas = obstacle(1, "gas_pipeline", F.createLineString(new Coordinate[]{c(0, 0), c(10, 0)}));
        assertEquals(2.0 + 0.40 / 2 + 0.510 / 2, gas.axisDistance(100), 1e-12);
        Obstacle cable = obstacle(2, "power_cable", F.createLineString(new Coordinate[]{c(0, 0), c(10, 0)}));
        assertEquals(2.0 + 0.20 / 2 + 0.510 / 2, cable.axisDistance(100), 1e-12);
    }

    @Test
    void segmentNearBuildingIsCheckedAlongItsWholeLength() {
        ObstacleSet set = set(obstacle(0, "oks", rect(0, 0, 10, 10)));
        // Далеко от здания (5,4 м > 5,255 м до оси ДУ 100) — можно.
        assertTrue(set.segmentClear(c(-20, -5.4), c(30, -5.4), 100, null));
        // Ближе допустимого хотя бы на середине отрезка — нельзя, даже если концы далеко.
        assertFalse(set.segmentClear(c(-20, -5.2), c(30, -5.2), 100, null));
        assertFalse(set.segmentClear(c(5, -50), c(5, -3), 100, null));
        // Насквозь через здание.
        assertFalse(set.segmentClear(c(5, -20), c(5, 30), 100, null));
        // Внутри здания.
        assertTrue(set.pointBlocked(c(5, 5), 100, null));
        assertFalse(set.pointBlocked(c(5, -8), 100, null));
    }

    @Test
    void largerDiameterNeedsLargerClearance() {
        ObstacleSet set = set(obstacle(0, "oks", rect(0, 0, 10, 10)));
        // 5,9 м от здания: для ДУ 100 достаточно (5,255), для ДУ 500 нужно 7 + 0,835.
        assertTrue(set.segmentClear(c(-20, -5.9), c(30, -5.9), 100, null));
        assertFalse(set.segmentClear(c(-20, -5.9), c(30, -5.9), 500, null));
        assertFalse(set.segmentClear(c(-20, -7.5), c(30, -7.5), 500, null));
        assertTrue(set.segmentClear(c(-20, -8.0), c(30, -8.0), 500, null));
    }

    @Test
    void exemptedObstacleDoesNotBlock() {
        Obstacle own = obstacle(0, "oks", rect(0, 0, 10, 10));
        ObstacleSet set = set(own);
        assertFalse(set.segmentClear(c(5, -20), c(5, 5), 100, null));
        assertTrue(set.segmentClear(c(5, -20), c(5, 5), 100, Set.of(own)));
        // Исключение действует только на собственное здание: чужое по-прежнему блокирует.
        Obstacle other = obstacle(1, "oks", rect(0, -40, 10, -30));
        ObstacleSet both = set(own, other);
        assertFalse(both.segmentClear(c(5, -60), c(5, 5), 100, Set.of(own)));
        assertEquals(other, both.firstViolation(c(5, -60), c(5, 5), 100, Set.of(own)));
    }

    /** Граница запретной зоны нигде не ближе к ограничению, чем требуется: дуги буфера не срезают отступ. */
    @Test
    void zoneBoundaryIsNeverCloserThanRequired() {
        // Г-образное здание с двориком-отверстием: острые и тупые углы, вогнутость и дырка.
        LinearRing shell = F.createLinearRing(new Coordinate[]{c(0, 0), c(40, 0), c(40, 15), c(15, 15), c(15, 40),
                c(0, 40), c(0, 0)});
        LinearRing hole = F.createLinearRing(new Coordinate[]{c(5, 5), c(12, 5), c(12, 12), c(5, 12), c(5, 5)});
        Obstacle o = obstacle(0, "oks", F.createPolygon(shell, new LinearRing[]{hole}));
        ObstacleSet set = set(o);
        for (int dn : new int[]{50, 100, 300, 500, 900, 1400}) {
            Geometry boundary = set.zone(o, dn).geometry.getBoundary();
            double required = o.axisDistance(dn);
            double minimal = Double.MAX_VALUE;
            for (Coordinate p : boundary.getCoordinates()) {
                minimal = Math.min(minimal, o.geometry.distance(F.createPoint(p)));
            }
            assertTrue(minimal >= required - 1e-9, "ДУ " + dn + ": граница зоны в " + minimal + " м, нужно " + required);
            assertTrue(minimal <= required + 0.05, "ДУ " + dn + ": зона шире нужного: " + minimal);
        }
    }

    @Test
    void zonesAreCachedPerDiameter() {
        Obstacle o = obstacle(0, "oks", rect(0, 0, 10, 10));
        ObstacleSet set = set(o);
        assertTrue(set.zone(o, 100) == set.zone(o, 100));
        assertTrue(set.zone(o, 100) != set.zone(o, 500));
    }

    @Test
    void emptySetNeverBlocks() {
        ObstacleSet set = new ObstacleSet(Collections.emptyList(), RuleSet.strict());
        assertTrue(set.segmentClear(c(0, 0), c(100, 100), 100, null));
    }

    // ---- Разбор входа ---------------------------------------------------------------------------------------------------

    private static PlanInput read(String json) throws Exception {
        return PlanInput.read(new ByteArrayInputStream(json.getBytes(StandardCharsets.UTF_8)), RuleSet.strict());
    }

    private static String point(String id, String type, String extra) {
        return "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Point\",\"coordinates\":[37.6,55.75]},\"properties\":{\"id\":"
                + id + ",\"object_type\":\"" + type + "\"" + extra + "}}";
    }

    @Test
    void readsTypedIdentifiersAndTargets() throws Exception {
        String json = "{\"type\":\"FeatureCollection\",\"features\":[" + point("1", "source", "") + ","
                + point("\"1\"", "heat_chamber", "") + "," + point("7", "oks_connection_point", ",\"flow_tph\":12.5") + "]}";
        PlanInput in = read(json);
        assertEquals(Id.number(1), in.sources.get(0).id);
        assertEquals(Id.text("1"), in.chambers.get(0).id);
        assertNotEquals(in.sources.get(0).id, in.chambers.get(0).id);
        assertEquals(12.5, in.targets.get(0).flow);
        // Координаты переведены в метры UTM: восток порядка 4·10^5, север порядка 6·10^6.
        double[] expected = Utm.forward(37.6, 55.75);
        assertEquals(expected[0], in.sources.get(0).xy.x, 1e-6);
        assertEquals(expected[1], in.sources.get(0).xy.y, 1e-6);
    }

    @Test
    void existingNetworkBecomesObstacleWithOwnEnvelope() throws Exception {
        String line = "{\"type\":\"Feature\",\"geometry\":{\"type\":\"LineString\",\"coordinates\":[[37.6,55.75],[37.601,55.75]]},"
                + "\"properties\":{\"id\":\"n1\",\"object_type\":\"heat_network\",\"diameter\":300}}";
        PlanInput in = read("{\"type\":\"FeatureCollection\",\"features\":[" + line + "]}");
        assertEquals(1, in.obstacles.size());
        Obstacle net = in.obstacles.get(0);
        assertTrue(net.isExistingNetwork());
        assertEquals(1.15, net.envelopeWidth, 1e-12);
        // Отступ 1,0 м + половина габарита сети 0,575 м + половина пары ДУ 100 (0,255 м).
        assertEquals(1.0 + 0.575 + 0.255, net.axisDistance(100), 1e-12);
    }

    private static String road(String props, String geometry) {
        return "{\"type\":\"Feature\",\"geometry\":" + geometry + ",\"properties\":{\"id\":\"r1\",\"object_type\":\"restriction\","
                + "\"restriction_type\":\"road\"" + props + "}}";
    }

    @Test
    void roadCenterlineWithWidthBecomesPolygonAndWithoutWidthStaysLine() throws Exception {
        String lineGeom = "{\"type\":\"LineString\",\"coordinates\":[[37.6,55.75],[37.601,55.75]]}";
        PlanInput withWidth = read("{\"type\":\"FeatureCollection\",\"features\":["
                + road(",\"geometry_role\":\"centerline\",\"width_m\":8.0", lineGeom) + "]}");
        Obstacle a = withWidth.obstacles.get(0);
        assertTrue(a.derivedFromCenterline);
        assertFalse(a.widthUnknown);
        assertTrue(a.crossable());
        assertEquals(2, a.geometry.getDimension());
        // Ширина полосы 8 м: площадь буфера оси длиной L равна 8·L (плоские концы).
        double[] p0 = Utm.forward(37.6, 55.75);
        double[] p1 = Utm.forward(37.601, 55.75);
        double length = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]);
        assertEquals(8.0 * length, a.geometry.getArea(), 0.5);

        PlanInput noWidth = read("{\"type\":\"FeatureCollection\",\"features\":[" + road("", lineGeom) + "]}");
        Obstacle b = noWidth.obstacles.get(0);
        assertTrue(b.widthUnknown);
        assertFalse(b.crossable());
        assertEquals(1, b.geometry.getDimension());
        assertTrue(noWidth.diagnostics.stream().anyMatch(d -> d.contains("ROAD_WIDTH_REQUIRED")));
    }

    @Test
    void roadWidthParsing() throws Exception {
        String geom = "{\"type\":\"LineString\",\"coordinates\":[[37.6,55.75],[37.601,55.75]]}";
        for (String bad : new String[]{",\"width\":\"2 lanes\"", ",\"width\":\"0\"", ",\"width_m\":null", ",\"width\":\"\""}) {
            PlanInput in = read("{\"type\":\"FeatureCollection\",\"features\":[" + road(bad, geom) + "]}");
            assertTrue(in.obstacles.get(0).widthUnknown, bad);
        }
        for (String good : new String[]{",\"width\":\"6\"", ",\"width\":\"7.5 m\"", ",\"width\":9"}) {
            PlanInput in = read("{\"type\":\"FeatureCollection\",\"features\":[" + road(good, geom) + "]}");
            assertFalse(in.obstacles.get(0).widthUnknown, good);
        }
    }

    @Test
    void unsupportedRestrictionTypesAreReportedNotUsed() throws Exception {
        String metro = "{\"type\":\"Feature\",\"geometry\":{\"type\":\"Polygon\",\"coordinates\":[[[37.6,55.75],[37.601,55.75],"
                + "[37.601,55.751],[37.6,55.75]]]},\"properties\":{\"id\":\"m\",\"object_type\":\"restriction\",\"restriction_type\":\"metro\"}}";
        PlanInput in = read("{\"type\":\"FeatureCollection\",\"features\":[" + metro + "]}");
        assertTrue(in.obstacles.isEmpty());
        assertTrue(in.diagnostics.get(0).contains("metro"));
    }
}
