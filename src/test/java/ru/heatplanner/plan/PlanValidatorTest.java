package ru.heatplanner.plan;

import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Polygon;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Валидатор должен находить намеренно сделанные нарушения и не ругаться на правильные трассы. */
class PlanValidatorTest {

    private static final GeometryFactory F = new GeometryFactory();
    private static final int DN = 100;

    private static Coordinate c(double x, double y) {
        return new Coordinate(x, y);
    }

    private static Polygon rect(double x1, double y1, double x2, double y2) {
        return F.createPolygon(new Coordinate[]{c(x1, y1), c(x2, y1), c(x2, y2), c(x1, y2), c(x1, y1)});
    }

    private static Obstacle obstacle(int i, String type, Geometry g) {
        return new Obstacle(i, Id.text(type + i), Rules.RESTRICTIONS.get(type), g, 0, "input", false, false);
    }

    private static Route.Leg leg(Coordinate a, Coordinate b) {
        return new Route.Leg(a, b, false, 1, List.of(), false, false);
    }

    private static Route route(String id, double flow, int dn, Route.Leg... legs) {
        PlanInput.Target target = new PlanInput.Target(Id.text(id), legs[0].from, flow);
        Start start = new Start(legs[0].from, null, Collections.emptySet(), null, target);
        Goal goal = new Goal(legs[legs.length - 1].to, Collections.emptySet(), null, "goal");
        return new Route(List.of(legs), start, goal, dn);
    }

    private static List<String> codes(List<Obstacle> obstacles, Route... routes) {
        ObstacleSet set = new ObstacleSet(obstacles, RuleSet.strict());
        List<String> out = new ArrayList<>();
        for (PlanValidator.Violation v : PlanValidator.validate(null, set, List.of(routes))) {
            out.add(v.code);
        }
        return out;
    }

    @Test
    void correctRouteHasNoViolations() {
        List<Obstacle> o = List.of(obstacle(0, "oks", rect(0, 20, 100, 40)));
        assertTrue(codes(o, route("a", 10, DN, leg(c(0, 0), c(100, 0)))).isEmpty());
    }

    @Test
    void tooCloseToBuildingIsReported() {
        List<Obstacle> o = List.of(obstacle(0, "oks", rect(0, 20, 100, 40)));
        assertTrue(codes(o, route("a", 10, DN, leg(c(0, 17), c(100, 17)))).contains("CLEARANCE"));
    }

    @Test
    void sharpTurnIsReported() {
        assertTrue(codes(List.of(), route("a", 10, DN, leg(c(0, 0), c(50, 0)), leg(c(50, 0), c(20, 30)))).contains("TURN"));
        assertTrue(codes(List.of(), route("a", 10, DN, leg(c(0, 0), c(50, 0)), leg(c(50, 0), c(80, 50)))).isEmpty());
    }

    @Test
    void capacityAndLengthAreChecked() {
        assertTrue(codes(List.of(), route("a", 100, DN, leg(c(0, 0), c(50, 0)))).contains("CAPACITY"));
        assertTrue(codes(List.of(), route("a", 10, DN, leg(c(0, 0), c(500, 0)))).contains("MAX_LENGTH"));
    }

    @Test
    void brokenChainIsReported() {
        assertTrue(codes(List.of(), route("a", 10, DN, leg(c(0, 0), c(50, 0)), leg(c(51, 0), c(80, 0)))).contains("GAP"));
    }

    private static Route crossing(Coordinate a, Coordinate b, double k, Obstacle road) {
        Route.Leg special = new Route.Leg(a, b, true, k, List.of(road), false, false);
        return route("a", 10, DN, special);
    }

    @Test
    void specialCrossingRulesAreChecked() {
        Obstacle road = obstacle(0, "road", rect(0, -5, 100, 5));
        List<Obstacle> o = List.of(road);
        assertTrue(codes(o, crossing(c(50, -8), c(50, 8), 1.60, road)).isEmpty());
        assertTrue(codes(o, crossing(c(50, -5.5), c(50, 5.5), 1.60, road)).contains("SPECIAL_EXTENT"));
        assertTrue(codes(o, crossing(c(40, -8), c(60, 8), 1.60, road)).contains("SPECIAL_ANGLE"));
        assertTrue(codes(o, crossing(c(50, -8), c(50, 8), 1.0, road)).contains("SPECIAL_K"));
    }

    @Test
    void ordinaryLegThroughRoadIsReported() {
        Obstacle road = obstacle(0, "road", rect(0, -5, 100, 5));
        assertTrue(codes(List.of(road), route("a", 10, DN, leg(c(50, -8), c(50, 8)))).contains("CLEARANCE"));
    }

    @Test
    void overlappingRoutesBreakTheTreeStructure() {
        Route a = route("a", 10, DN, leg(c(0, 0), c(100, 0)));
        Route b = route("b", 10, DN, leg(c(20, 30), c(30, 0)), leg(c(30, 0), c(100, 0)));
        assertTrue(codes(List.of(), a, b).contains("ROUTES_OVERLAP"));
        Route c = route("c", 10, DN, leg(c(0, 40), c(0, 50)), leg(c(0, 50), c(100, 50)));
        assertEquals(0, codes(List.of(), a, c).size());
    }
    /** Косой переход: 3 м отмеряются вдоль трассы от границы дороги, поэтому расстояние до полигона по перпендикуляру может быть меньше 3 м. */
    @Test
    void obliqueCrossingMeasuresTheExtentAlongTheLeg() {
        Obstacle road = obstacle(0, "road", rect(0, -5, 100, 5));
        double s60 = Math.sin(Math.toRadians(60));
        double c60 = Math.cos(Math.toRadians(60));
        Coordinate entry = c(50, -5);
        Coordinate exit = c(50 + 10 / s60 * c60, 5);
        Coordinate from = c(entry.x - 3 * c60, entry.y - 3 * s60);
        Coordinate to = c(exit.x + 3 * c60, exit.y + 3 * s60);
        Route.Leg special = new Route.Leg(from, to, true, 1.60, List.of(road), false, false);
        assertTrue(codes(List.of(road), route("a", 10, DN, special)).isEmpty(),
                codes(List.of(road), route("a", 10, DN, special)).toString());
    }
}