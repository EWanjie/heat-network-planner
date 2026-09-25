package ru.heatplanner.plan;

import org.junit.jupiter.api.Test;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Расчёт диаметров и стоимость дерева ветвей и его независимая проверка на небольших схемах в метрах. */
class TreeTest {

    private static final GeometryFactory F = new GeometryFactory();
    private static final ObstacleSet NONE = new ObstacleSet(List.of(), RuleSet.strict());

    private static Coordinate c(double x, double y) {
        return new Coordinate(x, y);
    }

    /** Существующая сеть: горизонтальный участок y = 0, ДУ 300. */
    private static Goals.Attach network() {
        PlanInput.Segment s = new PlanInput.Segment(Id.text("net"), 300, F.createLineString(new Coordinate[]{c(-200, 0), c(200, 0)}));
        return new Goals.Attach(Goals.Kind.NEW_CHAMBER, null, s, 300);
    }

    private static Branch branch(String id, double flow, int parent, double parentPos, Coordinate... pts) {
        List<Route.Leg> legs = new ArrayList<>();
        for (int i = 1; i < pts.length; i++) {
            legs.add(new Route.Leg(pts[i - 1], pts[i], false, 1, List.of(), false, false));
        }
        PlanInput.Target target = new PlanInput.Target(Id.text(id), pts[0], flow);
        Start start = new Start(pts[0], null, Collections.emptySet(), null, target);
        Coordinate end = pts[pts.length - 1];
        Goals.Attach existing = parent < 0 ? network() : null;
        Goal goal = new Goal(end, Collections.emptySet(), null, existing);
        return new Branch(target, new Route(legs, start, goal, 100), parent, existing, end, parentPos);
    }

    private static List<String> codes(List<Branch> bs, TreeEvaluator.Result ev) {
        List<String> out = new ArrayList<>();
        for (PlanValidator.Violation v : PlanValidator.validateTree(null, NONE, bs, ev)) {
            out.add(v.code);
        }
        return out;
    }

    private static List<Branch> trunkAndBranch(double flowA, double flowB) {
        Branch a = branch("A", flowA, -1, 0, c(50, 40), c(50, 0));
        Branch b = branch("B", flowB, 0, 20, c(80, 40), c(80, 20), c(50, 20));
        return List.of(a, b);
    }

    @Test
    void singleBranchGetsDiameterByFlowAndCostOfChamber() {
        List<Branch> bs = List.of(branch("A", 20, -1, 0, c(50, 40), c(50, 0)));
        TreeEvaluator.Result ev = TreeEvaluator.evaluate(null, NONE, bs);
        assertTrue(ev.feasible, ev.reason);
        assertEquals(1, ev.pieces.size());
        assertEquals(100, ev.pieces.get(0).dn);
        assertEquals(40 * Rules.PRICE[Rules.indexOfDn(100)] + Rules.chamberCost(300), ev.cost(), 1e-6);
        assertTrue(codes(bs, ev).isEmpty());
    }

    @Test
    void sharedTrunkCarriesSummedFlow() {
        List<Branch> bs = trunkAndBranch(20, 20);
        TreeEvaluator.Result ev = TreeEvaluator.evaluate(null, NONE, bs);
        assertTrue(ev.feasible, ev.reason);
        // Ствол A до узла несёт 20 т/ч (ДУ 100), после узла 40 т/ч (ДУ 125).
        assertEquals(100, ev.pieces.get(0).dn);
        assertEquals(125, ev.pieces.get(1).dn);
        assertEquals(20, ev.pieces.get(0).length, 1e-9);
        assertEquals(2, ev.chambers.size());
        assertTrue(codes(bs, ev).isEmpty(), codes(bs, ev).toString());
    }

    @Test
    void validatorFindsUndersizedPipe() {
        List<Branch> bs = List.of(branch("A", 20, -1, 0, c(50, 40), c(50, 0)));
        TreeEvaluator.Result fake = new TreeEvaluator.Result();
        fake.pieces.add(new TreeEvaluator.Piece(0, 0, c(50, 40), c(50, 0), 50, 20, 40, 1));
        assertTrue(codes(bs, fake).contains("CAPACITY"));
    }

    @Test
    void validatorFindsDiameterDecreasingTowardNetwork() {
        List<Branch> bs = trunkAndBranch(20, 20);
        TreeEvaluator.Result fake = new TreeEvaluator.Result();
        fake.pieces.add(new TreeEvaluator.Piece(0, 0, c(50, 40), c(50, 20), 125, 20, 20, 1));
        fake.pieces.add(new TreeEvaluator.Piece(0, 0, c(50, 20), c(50, 0), 100, 40, 20, 1));
        assertTrue(codes(bs, fake).contains("DN_DECREASES"));
    }

    @Test
    void validatorFindsTooLongRunOfOneDiameter() {
        List<Branch> bs = List.of(branch("A", 5, -1, 0, c(0, 500), c(0, 0)));
        TreeEvaluator.Result fake = new TreeEvaluator.Result();
        fake.pieces.add(new TreeEvaluator.Piece(0, 0, c(0, 500), c(0, 0), 50, 5, 500, 1));
        assertTrue(codes(bs, fake).contains("MAX_LENGTH"));
    }

    @Test
    void evaluatorRaisesDiameterWhenRunIsTooLong() {
        // ДУ 50 допускает 181 м; трасса 300 м с расходом 3 т/ч обязана получить ДУ больше.
        List<Branch> bs = List.of(branch("A", 3, -1, 0, c(0, 300), c(0, 0)));
        TreeEvaluator.Result ev = TreeEvaluator.evaluate(null, NONE, bs);
        assertTrue(ev.feasible, ev.reason);
        assertTrue(ev.pieces.get(0).dn > 50);
        assertTrue(ev.pieces.get(0).length <= Rules.MAX_LENGTH[Rules.indexOfDn(ev.pieces.get(0).dn)]);
        assertTrue(codes(bs, ev).isEmpty());
    }

    @Test
    void crossingBranchesOutsideAJunctionAreReported() {
        Branch a = branch("A", 10, -1, 0, c(50, 40), c(50, 0));
        Branch b = branch("B", 10, -1, 0, c(20, 30), c(80, 30), c(80, 0));
        List<Branch> bs = List.of(a, b);
        assertTrue(codes(bs, TreeEvaluator.evaluate(null, NONE, bs)).contains("ROUTES_OVERLAP"));
    }

    @Test
    void obtuseTurnAtJunctionIsReported() {
        Branch a = branch("A", 10, -1, 0, c(50, 40), c(50, 0));
        Branch b = branch("B", 10, 0, 20, c(80, 10), c(50, 20));
        List<Branch> bs = List.of(a, b);
        assertTrue(codes(bs, TreeEvaluator.evaluate(null, NONE, bs)).contains("JUNCTION_TURN"));
    }

    @Test
    void twoJunctionsTooCloseAreRejected() {
        Branch a = branch("A", 10, -1, 0, c(50, 40), c(50, 0));
        Branch b = branch("B", 10, 0, 20, c(80, 40), c(80, 20), c(50, 20));
        Branch d = branch("C", 10, 0, 20.5, c(20, 40), c(20, 19.5), c(50, 19.5));
        assertFalse(TreeEvaluator.evaluate(null, NONE, List.of(a, b, d)).feasible);
    }
}