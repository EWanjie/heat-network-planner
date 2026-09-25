package ru.heatplanner.plan;

import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Polygon;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Set;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Поиск трасс на небольших картах в метрах: результаты проверяются независимо от самого поиска. */
class RouterTest {

    private static final GeometryFactory F = new GeometryFactory();
    private static final int DN = 100;
    private static final double PRICE = Rules.PRICE[Rules.indexOfDn(DN)];

    private static Coordinate c(double x, double y) {
        return new Coordinate(x, y);
    }

    private static Polygon rect(double x1, double y1, double x2, double y2) {
        return F.createPolygon(new Coordinate[]{c(x1, y1), c(x2, y1), c(x2, y2), c(x1, y2), c(x1, y1)});
    }

    private static Obstacle obstacle(int i, String type, Geometry g) {
        return new Obstacle(i, Id.text(type + i), Rules.RESTRICTIONS.get(type), g, 0, "input", false, false);
    }

    private static Router router(Obstacle... obstacles) {
        return new Router(new ObstacleSet(List.of(obstacles), RuleSet.strict()), RuleSet.strict());
    }

    private static Start free(Coordinate at) {
        return new Start(at, null, Collections.emptySet(), null, null);
    }

    private static Goal goal(double x, double y) {
        return new Goal(c(x, y), Collections.emptySet(), null, "goal");
    }

    private static Router.Result run(Router r, Start s, Goal g, int dn, double budget) {
        return r.findRoute(s, List.of(g), dn, Router.Profile.BALANCED, budget);
    }

    /** Независимая проверка: каждый обычный участок на нужном расстоянии от всех ограничений, повороты не больше 90°. */
    private static void assertValid(Route route, List<Obstacle> obstacles, Set<Obstacle> exempt) {
        assertTrue(route.maxTurnDeg() <= 90 + 1e-6, "поворот " + route.maxTurnDeg());
        for (Route.Leg leg : route.legs) {
            if (leg.special || leg.terminal || leg.tieInApproach) {
                continue;
            }
            LineString seg = F.createLineString(new Coordinate[]{leg.from, leg.to});
            for (Obstacle o : obstacles) {
                if (exempt.contains(o)) {
                    continue;
                }
                double d = o.geometry.distance(seg);
                assertTrue(d >= o.axisDistance(route.dn) - 1e-6,
                        "участок ближе допустимого к " + o + ": " + d + " < " + o.axisDistance(route.dn));
            }
        }
    }

    @Test
    void freeSpaceGivesStraightRoute() {
        Router.Result r = run(router(), free(c(0, 0)), goal(100, 0), DN, 419);
        assertNotNull(r.route);
        assertEquals(1, r.route.legs.size());
        assertEquals(100.0, r.route.length, 1e-9);
        assertEquals(100 * PRICE, r.route.cost, 1e-6);
    }

    @Test
    void routeGoesAroundBuildingKeepingClearance() {
        Obstacle building = obstacle(0, "oks", rect(40, -10, 60, 10));
        Router.Result r = run(router(building), free(c(0, 0)), goal(100, 0), DN, 419);
        assertNotNull(r.route, r.status);
        assertValid(r.route, List.of(building), Set.of());
        // Прямая 100 м заблокирована зданием; обход по углам зон не длиннее ломаной через их вершины.
        double R = building.axisDistance(DN);
        double upper = 2 * Math.hypot(40 - R, 10 + R) + (20 + 2 * R);
        assertTrue(r.route.length > 100, "длина " + r.route.length);
        assertTrue(r.route.length <= upper + 1e-6, "длина " + r.route.length + " > " + upper);
    }

    @Test
    void longerDiameterNeedsWiderDetour() {
        Obstacle building = obstacle(0, "oks", rect(40, -10, 60, 10));
        Router router = router(building);
        double small = run(router, free(c(0, 0)), goal(100, 0), 100, 5000).route.length;
        double big = run(router, free(c(0, 0)), goal(100, 0), 900, 6518).route.length;
        assertTrue(big > small, "для ДУ 900 отступ 9 м, обход должен быть длиннее: " + big + " и " + small);
    }

    @Test
    void roadIsCrossedByOneStraightSpecialLegWithSurchargeAndBuffers() {
        Obstacle road = obstacle(0, "road", rect(40, -300, 60, 300));
        Router.Result r = run(router(road), free(c(0, 0)), goal(100, 0), DN, 419);
        assertNotNull(r.route, r.status);
        List<Route.Leg> special = new ArrayList<>();
        for (Route.Leg l : r.route.legs) {
            if (l.special) {
                special.add(l);
            }
        }
        assertEquals(1, special.size(), "специальный проход — ровно один прямой участок");
        // Полоса дороги 20 м плюс по 3 м за границей с каждой стороны.
        assertEquals(26.0, special.get(0).length(), 1e-6);
        assertEquals(1.60, special.get(0).kSpec);
        assertEquals(100.0, r.route.length, 1e-6);
        assertEquals(74 * PRICE + 26 * PRICE * 1.6, r.route.cost, 1e-3);
        assertEquals(37.0, special.get(0).from.x, 1e-6);
        assertEquals(63.0, special.get(0).to.x, 1e-6);
    }

    @Test
    void roadCrossingNeedsAtLeast45Degrees() {
        // Полоса шириной 10 м, идущая под 30° к оси x: прямая по оси x пересекла бы её под 30° — так нельзя.
        double ang = Math.toRadians(30);
        double[] along = {Math.cos(ang), Math.sin(ang)};
        double[] nrm = {-along[1], along[0]};
        Coordinate mid = c(50, 0);
        Coordinate[] ring = new Coordinate[5];
        double[][] signs = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}, {-1, -1}};
        for (int i = 0; i < 5; i++) {
            ring[i] = c(mid.x + along[0] * 800 * signs[i][0] + nrm[0] * 5 * signs[i][1],
                    mid.y + along[1] * 800 * signs[i][0] + nrm[1] * 5 * signs[i][1]);
        }
        Obstacle road = obstacle(0, "road", F.createPolygon(ring));
        Router.Result r = run(router(road), free(c(0, 0)), goal(100, 0), DN, 419);
        assertNotNull(r.route, r.status);
        assertTrue(r.route.length > 100.5, "по прямой нельзя: длина " + r.route.length);
        for (Route.Leg l : r.route.legs) {
            if (l.special) {
                double a = Angles.acuteDeg(Angles.unit(l.from, l.to), along);
                assertTrue(a >= 45 - 1e-6, "угол пересечения " + a);
            }
        }
        assertTrue(r.route.maxTurnDeg() <= 90 + 1e-6);
    }

    @Test
    void roadOfUnknownWidthCannotBeCrossed() {
        Obstacle road = new Obstacle(0, Id.text("r"), Rules.RESTRICTIONS.get("road"),
                F.createLineString(new Coordinate[]{c(50, -40), c(50, 40)}), 0, "OpenStreetMap", true, false);
        Router router = router(road);
        // Дорога длиной 80 м: обход возможен, пересечение — нет.
        Router.Result r = run(router, free(c(0, 0)), goal(100, 0), DN, 419);
        assertNotNull(r.route, r.status);
        assertTrue(r.route.legs.stream().noneMatch(l -> l.special));
        assertTrue(r.route.length > 100);
    }

    @Test
    void ownBuildingApproachExitsThroughNearestWallStraight() {
        Obstacle own = obstacle(0, "oks", rect(0, 0, 20, 20));
        ObstacleSet set = new ObstacleSet(List.of(own), RuleSet.strict());
        PlanInput.Target t = new PlanInput.Target(Id.text("cp"), c(10, 10), 20);
        List<Start> starts = Start.forTarget(t, set, DN, RuleSet.strict());
        assertFalse(starts.isEmpty());
        for (Start s : starts) {
            assertEquals(c(10, 10), s.terminalPoint);
            assertTrue(s.exempt.contains(own));
            // Старт лежит вне зоны отступа собственного полигона.
            assertFalse(set.zone(own, DN).geometry.intersects(F.createPoint(s.origin)));
        }
        Router router = new Router(set, RuleSet.strict());
        Router.Result r = router.findBest(starts, List.of(goal(100, 10)), DN, Router.Profile.BALANCED, 419);
        assertNotNull(r.route, r.status);
        assertTrue(r.route.legs.get(0).terminal);
        assertEquals(c(10, 10), r.route.legs.get(0).from);
        // Путь на восток по прямой: 90 м от точки подключения до врезки.
        assertEquals(90.0, r.route.length, 1e-6);
        assertValid(r.route, List.of(own), Set.of(own));
    }

    @Test
    void ownApproachIsRejectedWhereAnotherBuildingBlocksIt() {
        Obstacle own = obstacle(0, "oks", rect(0, 0, 20, 20));
        Obstacle neighbour = obstacle(1, "oks", rect(24, 0, 44, 20));
        ObstacleSet set = new ObstacleSet(List.of(own, neighbour), RuleSet.strict());
        PlanInput.Target t = new PlanInput.Target(Id.text("cp"), c(10, 10), 20);
        List<Start> starts = Start.forTarget(t, set, DN, RuleSet.strict());
        assertFalse(starts.isEmpty());
        for (Start s : starts) {
            // На восток идти нельзя: там чужое здание вплотную, выход из зоны своего попадает в зону чужого.
            assertFalse(s.direction[0] > 0.9, "направление " + s.direction[0] + "," + s.direction[1]);
        }
        Router.Result r = new Router(set, RuleSet.strict())
                .findBest(starts, List.of(goal(100, 10)), DN, Router.Profile.BALANCED, 5000);
        assertNotNull(r.route, r.status);
        assertValid(r.route, List.of(own, neighbour), Set.of(own));
    }

    @Test
    void tieInApproachMustNotRunAlongExistingPipe() {
        // Существующая сеть вдоль оси x; врезка в точку (50, 0).
        Obstacle pipe = new Obstacle(0, Id.text("n"), Rules.RESTRICTIONS.get("heat_network"),
                F.createLineString(new Coordinate[]{c(0, 0), c(100, 0)}), Rules.pairWidth(300), "existing_network", false, false);
        Router router = router(pipe);
        Goal tie = new Goal(c(50, 0), Set.of(pipe), new double[]{1, 0}, "tie");
        // Сверху, под прямым углом: 30 м.
        Router.Result steep = router.findRoute(free(c(50, 30)), List.of(tie), DN, Router.Profile.COMPACT, 419);
        assertNotNull(steep.route, steep.status);
        assertEquals(30.0, steep.route.length, 1e-6);
        // Издалека и полого (5,7° к линии) напрямую нельзя: угол подхода меньше 45°.
        Router.Result shallow = router.findRoute(free(c(80, 3)), List.of(tie), DN, Router.Profile.COMPACT, 419);
        assertNotNull(shallow.route, shallow.status);
        assertTrue(shallow.route.length > Math.hypot(30, 3) + 0.5, "длина " + shallow.route.length);
        Route.Leg last = shallow.route.legs.get(shallow.route.legs.size() - 1);
        assertTrue(last.tieInApproach);
        assertTrue(Angles.acuteDeg(Angles.unit(last.from, last.to), new double[]{1, 0}) >= 45 - 1e-6);
        assertTrue(shallow.route.maxTurnDeg() <= 90 + 1e-6);
    }

    @Test
    void routeLongerThanDiameterLimitIsNotAccepted() {
        Router router = router();
        // ДУ 50: предельная длина 181 м; 200 м не проходят, ДУ 65 (245 м) проходят.
        assertNull(run(router, free(c(0, 0)), goal(200, 0), 50, Rules.MAX_LENGTH[Rules.indexOfDn(50)]).route);
        assertNotNull(run(router, free(c(0, 0)), goal(200, 0), 65, Rules.MAX_LENGTH[Rules.indexOfDn(65)]).route);
    }

    @Test
    void enclosedStartHasNoRoute() {
        // Замкнутая стена из парков вокруг старта: выйти нельзя.
        List<Obstacle> walls = new ArrayList<>();
        walls.add(obstacle(0, "park", rect(-30, 20, 30, 30)));
        walls.add(obstacle(1, "park", rect(-30, -30, 30, -20)));
        walls.add(obstacle(2, "park", rect(20, -30, 30, 30)));
        walls.add(obstacle(3, "park", rect(-30, -30, -20, 30)));
        Router.Result r = run(router(walls.toArray(new Obstacle[0])), free(c(0, 0)), goal(100, 0), DN, 5000);
        assertNull(r.route);
    }

    @Test
    void turnsNeverExceedNinetyDegreesInAPocket() {
        // Здание в форме «П», открытое на запад; старт в кармане, цель за восточной стеной.
        Obstacle top = obstacle(0, "oks", rect(0, 30, 60, 40));
        Obstacle bottom = obstacle(1, "oks", rect(0, -40, 60, -30));
        Obstacle back = obstacle(2, "oks", rect(50, -40, 60, 40));
        Router.Result r = run(router(top, bottom, back), free(c(25, 0)), goal(100, 0), DN, 5000);
        assertNotNull(r.route, r.status);
        assertValid(r.route, List.of(top, bottom, back), Set.of());
        assertTrue(r.route.maxTurnDeg() <= 90 + 1e-6);
        assertTrue(r.route.length > 75, "нужен обход вокруг здания, длина " + r.route.length);
    }

    /** Ближайшая к точке стена смотрит на соседнее здание: подход туда невозможен, дальняя сторона свободна. */
    @Test
    void ownApproachModesDifferWhenNearestWallIsBlocked() {
        Obstacle own = obstacle(0, "oks", rect(0, 0, 20, 20));
        Obstacle south = obstacle(1, "oks", rect(-10, -14, 30, -8));
        ObstacleSet set = new ObstacleSet(List.of(own, south), RuleSet.strict());
        PlanInput.Target t = new PlanInput.Target(Id.text("cp"), c(10, 4), 20);
        // Строго по приложению: только ближайшая (южная) граница; она упирается в чужое здание.
        List<Start.Refusal> why = new java.util.ArrayList<>();
        assertTrue(Start.forTarget(t, set, DN, RuleSet.strict(), why).isEmpty());
        assertEquals(1, why.size());
        assertEquals(Start.Refusal.Code.EXIT_IN_OTHER_ZONE, why.get(0).code);
        assertTrue(south == why.get(0).blocker);
        // Запасной режим: ближайшая из остальных допустимых границ, и это отмечено.
        List<Start> fallback = Start.forTarget(t, set, DN,
                RuleSet.strict().withOwnApproach(RuleSet.OwnApproachMode.NEAREST_VALID));
        assertFalse(fallback.isEmpty());
        for (Start s : fallback) {
            assertFalse(s.nearestBoundary);
            assertFalse(set.zone(south, DN).geometry.intersects(F.createPoint(s.origin)));
        }
    }

    /** Точка в центре здания одинаково близка ко всем стенам: строгий режим рассматривает их все. */
    @Test
    void strictModeKeepsAllEquallyNearestWalls() {
        Obstacle own = obstacle(0, "oks", rect(0, 0, 20, 20));
        ObstacleSet set = new ObstacleSet(List.of(own), RuleSet.strict());
        PlanInput.Target t = new PlanInput.Target(Id.text("cp"), c(10, 10), 20);
        List<Start> starts = Start.forTarget(t, set, DN, RuleSet.strict());
        assertEquals(3, starts.size(), "берётся не больше трёх различающихся направлений");
        assertTrue(starts.stream().allMatch(s -> s.nearestBoundary));
    }

    /** Ближайшая граница — узкий внутренний двор: выход из зоны собственного здания во дворе невозможен. */
    @Test
    void refusalNamesCourtyardWhenNearestBoundaryIsAHole() {
        org.locationtech.jts.geom.LinearRing shell = F.createLinearRing(new Coordinate[]{
                c(0, 0), c(20, 0), c(20, 20), c(0, 20), c(0, 0)});
        org.locationtech.jts.geom.LinearRing hole = F.createLinearRing(new Coordinate[]{
                c(8, 8), c(12, 8), c(12, 12), c(8, 12), c(8, 8)});
        Obstacle own = obstacle(0, "oks", F.createPolygon(shell, new org.locationtech.jts.geom.LinearRing[]{hole}));
        ObstacleSet set = new ObstacleSet(List.of(own), RuleSet.strict());
        PlanInput.Target t = new PlanInput.Target(Id.text("cp"), c(10, 7), 20);
        List<Start.Refusal> why = new java.util.ArrayList<>();
        assertTrue(Start.forTarget(t, set, DN, RuleSet.strict(), why).isEmpty());
        assertEquals(Start.Refusal.Code.NEAREST_BOUNDARY_IN_COURTYARD, why.get(0).code);
    }
}