package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Оценка дерева ветвей: расходы участков, подбор диаметров, проверка длины, отступов на итоговом ДУ и стоимость.
 *
 * Расход участка — сумма расходов всех точек, подключённых ниже по течению. Диаметр участка — наименьший по
 * пропускной способности; затем он не убывает в сторону врезки, а любая непрерывная часть одного диаметра на пути
 * от точки подключения к сети не длиннее предельной для этого диаметра (иначе диаметр увеличивается). Путь общего
 * ствола учитывается в каждом пути, проходящем по нему.
 */
public final class TreeEvaluator {

    public static final class Piece {
        public final int branch;
        public final int leg;
        public final Coordinate from;
        public final Coordinate to;
        public final int dn;
        public final double flow;
        public final double length;
        public final double cost;

        Piece(int branch, int leg, Coordinate from, Coordinate to, int dn, double flow, double length, double cost) {
            this.branch = branch;
            this.leg = leg;
            this.from = from;
            this.to = to;
            this.dn = dn;
            this.flow = flow;
            this.length = length;
            this.cost = cost;
        }
    }

    public static final class Chamber {
        public final Coordinate xy;
        public final boolean existing;
        public final int dn;
        public final double cost;
        /** Первая из ветвей, заканчивающихся в камере. */
        public final int branch;
        /** Все ветви, заканчивающиеся в камере (в узле новой сети их до двух: вместе с проходящим стволом — четыре примыкания). */
        public final List<Integer> branches;

        Chamber(Coordinate xy, boolean existing, int dn, double cost, int branch, List<Integer> branches) {
            this.xy = xy;
            this.existing = existing;
            this.dn = dn;
            this.cost = cost;
            this.branch = branch;
            this.branches = branches;
        }
    }

    public static final class Result {
        public boolean feasible = true;
        public String reason = "";
        public double pipeCost;
        public double chamberCost;
        public double tieInCost;
        public double length;
        public int existingTieIns;
        public final List<Piece> pieces = new ArrayList<>();
        public final List<Chamber> chambers = new ArrayList<>();

        public double cost() {
            return pipeCost + chamberCost + tieInCost;
        }

        Result fail(String why) {
            feasible = false;
            reason = why;
            return this;
        }
    }

    private static final double EPS = 1e-6;
    /** Две врезки на одной ветви ближе этого расстояния (м) — одна камера на четыре участка недопустима для такой пары. */
    private static final double MIN_JOIN_GAP_M = 2.0;
    /** Врезки ближе этого расстояния (м) — один узел: ветви заканчиваются в одной камере. */
    private static final double SAME_NODE_M = 0.05;
    /** В узле новой сети проходящий ствол даёт два примыкания, значит своих ветвей не более двух (всего четыре). */
    private static final int MAX_BRANCHES_PER_NODE = 2;

    private TreeEvaluator() {
    }

    public static Result evaluate(PlanInput in, ObstacleSet obstacles, List<Branch> bs) {
        Result res = new Result();
        int n = bs.size();
        double[] sub = new double[n];
        for (int i = 0; i < n; i++) {
            sub[i] = bs.get(i).target.flow;
        }
        for (int i = n - 1; i >= 0; i--) {
            if (bs.get(i).parent >= 0) {
                sub[bs.get(i).parent] += sub[i];
            }
        }
        // Точки разбиения каждой ветви: начало, конец и места врезок других ветвей.
        List<List<Integer>> children = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            children.add(new ArrayList<>());
        }
        for (int i = 0; i < n; i++) {
            if (bs.get(i).parent >= 0) {
                children.get(bs.get(i).parent).add(i);
            }
        }
        double[][] bp = new double[n][];
        // Положение врезки каждой дочерней ветви после объединения близких врезок в один узел.
        double[] jpos = new double[n];
        Map<Integer, List<Integer>> nodeMembers = new HashMap<>();
        for (int b = 0; b < n; b++) {
            List<Double> list = new ArrayList<>();
            list.add(0.0);
            List<Integer> kids = new ArrayList<>(children.get(b));
            kids.sort(java.util.Comparator.comparingDouble(c -> bs.get(c).parentPos));
            double groupStart = Double.NEGATIVE_INFINITY;
            List<Integer> group = null;
            for (int c : kids) {
                double pos = bs.get(c).parentPos;
                if (group != null && pos - groupStart < SAME_NODE_M) {
                    if (group.size() >= MAX_BRANCHES_PER_NODE) {
                        return res.fail("в узле больше четырёх примыкающих участков");
                    }
                    group.add(c);
                    jpos[c] = groupStart;
                    continue;
                }
                if (group != null && pos - groupStart < MIN_JOIN_GAP_M) {
                    return res.fail("два узла на одной ветви ближе " + MIN_JOIN_GAP_M + " м");
                }
                if (!bs.get(b).canJoinAt(pos)) {
                    return res.fail("врезка на участке, куда её ставить нельзя");
                }
                groupStart = pos;
                group = new ArrayList<>();
                group.add(c);
                jpos[c] = pos;
                nodeMembers.put(c, group);
                list.add(pos);
            }
            list.add(bs.get(b).total);
            bp[b] = list.stream().mapToDouble(Double::doubleValue).toArray();
        }
        int[][] dn = new int[n][];
        double[][] flow = new double[n][];
        for (int b = 0; b < n; b++) {
            int segs = bp[b].length - 1;
            dn[b] = new int[segs];
            flow[b] = new double[segs];
            for (int k = 0; k < segs; k++) {
                double f = bs.get(b).target.flow;
                for (int c : children.get(b)) {
                    if (jpos[c] <= bp[b][k] + EPS) {
                        f += sub[c];
                    }
                }
                flow[b][k] = f;
                int idx = Rules.indexForFlow(f);
                if (idx < 0) {
                    return res.fail("расход " + f + " т/ч больше пропускной способности самого большого ДУ");
                }
                dn[b][k] = idx;
            }
        }
        // Диаметры: не убывают к врезке, непрерывная часть одного диаметра не длиннее предельной.
        List<List<int[]>> paths = new ArrayList<>();
        for (int b = 0; b < n; b++) {
            List<int[]> path = new ArrayList<>();
            int cur = b;
            int startK = 0;
            while (true) {
                for (int k = startK; k < dn[cur].length; k++) {
                    path.add(new int[]{cur, k});
                }
                Branch br = bs.get(cur);
                if (br.parent < 0) {
                    break;
                }
                startK = indexOf(bp[br.parent], jpos[cur]);
                cur = br.parent;
            }
            paths.add(path);
        }
        for (int iter = 0; iter < 1000; iter++) {
            boolean changed = false;
            for (List<int[]> path : paths) {
                int run = -1;
                for (int[] s : path) {
                    if (dn[s[0]][s[1]] < run) {
                        dn[s[0]][s[1]] = run;
                        changed = true;
                    } else {
                        run = dn[s[0]][s[1]];
                    }
                }
            }
            if (!changed) {
                outer:
                for (List<int[]> path : paths) {
                    int i = 0;
                    while (i < path.size()) {
                        int j = i;
                        double len = 0;
                        int d = dn[path.get(i)[0]][path.get(i)[1]];
                        while (j < path.size() && dn[path.get(j)[0]][path.get(j)[1]] == d) {
                            int[] s = path.get(j);
                            len += bp[s[0]][s[1] + 1] - bp[s[0]][s[1]];
                            j++;
                        }
                        if (len > Rules.MAX_LENGTH[d] + EPS) {
                            if (d + 1 >= Rules.DN.length) {
                                return res.fail("длина непрерывной части не укладывается даже в самый большой ДУ");
                            }
                            for (int q = i; q < j; q++) {
                                dn[path.get(q)[0]][path.get(q)[1]] = d + 1;
                            }
                            changed = true;
                            break outer;
                        }
                        i = j;
                    }
                }
            }
            if (!changed) {
                break;
            }
            if (iter == 999) {
                return res.fail("подбор диаметров не сошёлся");
            }
        }
        // Участки труб на итоговом диаметре.
        for (int b = 0; b < n; b++) {
            Branch br = bs.get(b);
            for (int k = 0; k < dn[b].length; k++) {
                int dnv = Rules.DN[dn[b][k]];
                double price = Rules.PRICE[dn[b][k]];
                for (int li = 0; li < br.route.legs.size(); li++) {
                    Route.Leg leg = br.route.legs.get(li);
                    double l0 = br.legStart[li];
                    double l1 = br.legStart[li + 1];
                    double a = Math.max(bp[b][k], l0);
                    double c = Math.min(bp[b][k + 1], l1);
                    if (c - a <= 1e-9) {
                        continue;
                    }
                    Coordinate from = lerp(leg, (a - l0) / (l1 - l0));
                    Coordinate to = lerp(leg, (c - l0) / (l1 - l0));
                    Set<Obstacle> exempt = new HashSet<>();
                    if (leg.terminal) {
                        exempt.addAll(br.route.start.exempt);
                    }
                    if (leg.tieInApproach) {
                        exempt.addAll(br.route.goal.exempt);
                    }
                    if (leg.special) {
                        exempt.addAll(leg.crossed);
                    }
                    Obstacle bad = obstacles.firstViolation(from, to, dnv, exempt);
                    if (bad != null) {
                        return res.fail("на ДУ " + dnv + " участок ветви точки " + br.target.id.value() + " нарушает отступ до " + bad);
                    }
                    double length = c - a;
                    double cost = length * price * leg.kSpec;
                    res.pipeCost += cost;
                    res.length += length;
                    res.pieces.add(new Piece(b, li, from, to, dnv, flow[b][k], length, cost));
                }
            }
        }
        // Камеры и врезки.
        Map<Object, Integer> ties = new HashMap<>();
        for (int b = 0; b < n; b++) {
            Branch br = bs.get(b);
            int lastDn = Rules.DN[dn[b][dn[b].length - 1]];
            if (br.parent < 0) {
                if (br.existing.kind == Goals.Kind.EXISTING_CHAMBER) {
                    res.tieInCost += Rules.EXISTING_CHAMBER_TIE_IN;
                    res.existingTieIns++;
                    res.chambers.add(new Chamber(br.attach, true, lastDn, Rules.EXISTING_CHAMBER_TIE_IN, b, List.of(b)));
                    int used = ties.merge(br.existing.chamber, 1, Integer::sum);
                    if (in != null && Goals.adjacency(in, br.existing.chamber) + used > Rules.MAX_CHAMBER_SEGMENTS) {
                        return res.fail("к существующей камере примыкает больше четырёх участков");
                    }
                } else {
                    double cost = Rules.chamberCost(Math.max(lastDn, br.existing.existingDn));
                    res.chamberCost += cost;
                    res.chambers.add(new Chamber(br.attach, false, Math.max(lastDn, br.existing.existingDn), cost, b, List.of(b)));
                }
            } else {
                List<Integer> members = nodeMembers.get(b);
                if (members == null) {
                    continue; // второй ветви узла: камера уже записана вместе с первой
                }
                int p = br.parent;
                int k = indexOf(bp[p], jpos[b]);
                int max = Math.max(Rules.DN[dn[p][k - 1]], Rules.DN[dn[p][k]]);
                for (int m : members) {
                    max = Math.max(max, Rules.DN[dn[m][dn[m].length - 1]]);
                }
                double cost = Rules.chamberCost(max);
                res.chamberCost += cost;
                res.chambers.add(new Chamber(br.attach, false, max, cost, b, new ArrayList<>(members)));
            }
        }
        return res;
    }

    private static Coordinate lerp(Route.Leg leg, double t) {
        return new Coordinate(leg.from.x + (leg.to.x - leg.from.x) * t, leg.from.y + (leg.to.y - leg.from.y) * t);
    }

    private static int indexOf(double[] bp, double pos) {
        for (int i = 0; i < bp.length; i++) {
            if (Math.abs(bp[i] - pos) < 1e-6) {
                return i;
            }
        }
        throw new IllegalStateException("нет точки разбиения " + pos);
    }
}