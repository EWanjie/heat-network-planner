package ru.heatplanner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.TreeSet;

/**
 * Превращает дерево новой сети в готовый расчёт по правилам технического приложения:
 * расходы по участкам (сумма ОКС ниже), диаметры, предельная длина одного диаметра, зоны специального прохода,
 * камеры, врезки, реконструкция существующей сети и камер, штраф за неподключённые ОКС.
 *
 * Вызывается сотни раз при поиске вариантов, поэтому работает только с уже готовой геометрией дерева.
 */
final class Evaluator {

    private final PlanModel model;
    private final Tree tree;
    private final Solution sol = new Solution();
    private final Map<Tree.Node, Double> flowBelow = new IdentityHashMap<>();
    /** Первый и последний индекс диаметра каждого участка дерева: нужны для расчёта диаметра камер. */
    private final Map<Tree.Edge, int[]> edgeDn = new IdentityHashMap<>();

    private Evaluator(PlanModel model, Tree tree) {
        this.model = model;
        this.tree = tree;
    }

    static Solution evaluate(Tree tree, PlanModel model) {
        return new Evaluator(model, tree).run();
    }

    private Solution run() {
        for (Tree.Node n : tree.nodes) {
            if (n.isTie()) {
                flow(n);
            }
        }
        for (Tree.Node n : tree.nodes) {
            if (n.isTie()) {
                for (Tree.Edge e : n.out) {
                    edge(e, -1, 0);
                }
            }
        }
        if (sol.invalid) {
            return sol;
        }
        Map<PlanModel.Segment, List<double[]>> finalDn = reconstruction();
        if (sol.invalid) {
            return sol;
        }
        chambersAndTieIns(finalDn);
        for (PlanModel.Target t : model.targets) {
            if (!tree.contains(t)) {
                sol.unconnected.add(t);
                sol.penalty += Rules.penalty(t.flow);
            }
        }
        for (Solution.Piece p : sol.pieces) {
            sol.constructionCost += p.cost;
            sol.newLength += p.length;
        }
        for (Solution.NewChamber c : sol.chambers) {
            sol.chamberCost += c.cost;
        }
        sol.tieInCost = sol.tieIns.size() * Rules.TIE_IN_COST;
        for (Solution.Recon r : sol.recons) {
            sol.reconCost += r.cost;
            sol.reconLength += r.length;
        }
        for (Solution.ChamberRecon r : sol.chamberRecons) {
            sol.chamberReconCost += r.cost;
        }
        return sol;
    }

    // ---- Расходы --------------------------------------------------------------------------------------------------

    private double flow(Tree.Node n) {
        double f = 0;
        if (n.kind == Tree.Kind.TERMINAL) {
            f = n.target.flow;
        } else {
            for (Tree.Edge e : n.out) {
                f += flow(e.to);
            }
        }
        flowBelow.put(n, f);
        return f;
    }

    // ---- Новые участки: диаметр, предельная длина, специальный проход ---------------------------------------------

    /**
     * Режет трубу на участки с постоянными параметрами. Граница ставится там, где меняется способ прокладки
     * (заход в зону специального прохода) или диаметр (исчерпана предельная длина текущего диаметра).
     * prevIdx/prevRun — диаметр и пройденная длина непрерывной части того же диаметра на входе в трубу.
     */
    private void edge(Tree.Edge e, int prevIdx, double prevRun) {
        double flow = flowBelow.get(e.to);
        int idx0 = Rules.indexForFlow(flow);
        if (idx0 < 0) {
            sol.invalid = true;
            return;
        }
        // Спаны: чередование обычных участков и зон специального прохода.
        List<double[]> spans = new ArrayList<>();
        List<Zones.Zone> spanZone = new ArrayList<>();
        double d = 0;
        for (Zones.Zone z : e.zones) {
            if (z.from > d + 1e-6) {
                spans.add(new double[]{d, z.from});
                spanZone.add(null);
            }
            spans.add(new double[]{Math.max(d, z.from), z.to});
            spanZone.add(z);
            d = z.to;
        }
        if (e.length > d + 1e-6) {
            spans.add(new double[]{d, e.length});
            spanZone.add(null);
        }

        int idx = idx0;
        double run = prevIdx == idx0 ? prevRun : 0;
        List<Solution.Piece> made = new ArrayList<>();
        List<String> reasons = new ArrayList<>();
        int firstIdx = -1;
        for (int s = 0; s < spans.size(); s++) {
            double from = spans.get(s)[0];
            double to = spans.get(s)[1];
            Zones.Zone zone = spanZone.get(s);
            while (to - from > 1e-6) {
                double room = Rules.MAX_LENGTH[idx] - run;
                if (room <= 1e-6) {
                    if (idx == Rules.DN.length - 1) {
                        room = to - from;
                    } else {
                        idx++;
                        run = 0;
                        // Диаметр сменился посреди трубы: граница участков — технический узел.
                        if (!made.isEmpty() && reasons.get(made.size() - 1) == null) {
                            reasons.set(made.size() - 1, "смена диаметра (предельная длина)");
                        }
                        continue;
                    }
                }
                double take = Math.min(to - from, room);
                Solution.Piece p = new Solution.Piece();
                p.pts = Geo.sub(e.pts, from, from + take);
                p.length = take;
                p.flow = flow;
                p.dn = Rules.DN[idx];
                p.special = zone != null;
                p.k = zone == null ? 1 : zone.k;
                p.zoneLabel = zone == null ? null : zone.rule.title;
                p.cost = take * Rules.COST_NEW[idx] * p.k;
                if (firstIdx < 0) {
                    firstIdx = idx;
                }
                // Причина границы после этого участка: если продолжение в другой зоне — смена способа прокладки.
                made.add(p);
                reasons.add(null);
                run += take;
                from += take;
            }
            if (!made.isEmpty() && s + 1 < spans.size() && reasons.get(made.size() - 1) == null) {
                reasons.set(made.size() - 1, "смена способа прокладки");
            }
        }
        // Связываем участки узлами: начало трубы, технические узлы, конец трубы.
        Object prev = e.from;
        for (int i = 0; i < made.size(); i++) {
            Solution.Piece p = made.get(i);
            p.start = prev;
            if (i == made.size() - 1) {
                p.end = e.to;
            } else {
                Solution.TechNode t = new Solution.TechNode(p.pts[p.pts.length - 1],
                        reasons.get(i) == null ? "смена параметров участка" : reasons.get(i));
                sol.techNodes.add(t);
                p.end = t;
            }
            prev = p.end;
            sol.pieces.add(p);
        }
        edgeDn.put(e, new int[]{firstIdx < 0 ? idx0 : firstIdx, idx});
        for (Tree.Edge child : e.to.out) {
            edge(child, idx, run);
        }
    }

    // ---- Реконструкция существующей сети (раздел 7) ---------------------------------------------------------------

    /**
     * Возвращает для каждого затронутого существующего участка его итоговые диаметры по частям {от, до, DN}:
     * камеры и врезки смотрят на диаметр той части участка, которая к ним примыкает, а не на весь участок.
     */
    private Map<PlanModel.Segment, List<double[]>> reconstruction() {
        Map<PlanModel.Segment, Double> whole = new HashMap<>();
        Map<PlanModel.Segment, List<double[]>> partial = new HashMap<>();
        for (Tree.Node tie : tree.nodes) {
            if (!tie.isTie()) {
                continue;
            }
            double g = flowBelow.get(tie);
            PlanModel.Segment start;
            if (tie.kind == Tree.Kind.TIE_SEGMENT) {
                // Врезка внутрь участка: дополнительный расход действует только на часть участка от точки врезки к источнику.
                partial.computeIfAbsent(tie.segment, k -> new ArrayList<>()).add(new double[]{tie.along, g});
                start = tie.segment.upSeg;
            } else {
                start = tie.chamber.upSeg;
            }
            for (PlanModel.Segment s = start; s != null; s = s.upSeg) {
                whole.merge(s, g, Double::sum);
            }
        }
        java.util.Set<PlanModel.Segment> affected = new java.util.LinkedHashSet<>(whole.keySet());
        affected.addAll(partial.keySet());

        Map<PlanModel.Segment, List<double[]>> finalDn = new HashMap<>();
        for (PlanModel.Segment s : affected) {
            double w = whole.getOrDefault(s, 0.0);
            List<double[]> parts = partial.getOrDefault(s, new ArrayList<>());
            TreeSet<Double> bounds = new TreeSet<>();
            bounds.add(0.0);
            for (double[] p : parts) {
                bounds.add(Math.min(p[0], s.length));
            }
            if (w > 0) {
                bounds.add(s.length);
            }
            List<Double> b = new ArrayList<>(bounds);
            int maxReq = Rules.indexOfDn(s.dn);
            double maxAdd = 0;
            List<double[]> parts2 = new ArrayList<>();
            finalDn.put(s, parts2);
            Solution.Recon open = null;
            for (int i = 0; i + 1 < b.size(); i++) {
                double lo = b.get(i);
                double hi = b.get(i + 1);
                if (hi - lo < 1e-6) {
                    continue;
                }
                double add = w;
                for (double[] p : parts) {
                    if (p[0] >= hi - 1e-9) {
                        add += p[1];
                    }
                }
                if (add <= 0) {
                    continue;
                }
                int req = Rules.indexForFlow(s.flowEff + add);
                if (req < 0) {
                    sol.invalid = true;
                    return finalDn;
                }
                maxAdd = Math.max(maxAdd, add);
                maxReq = Math.max(maxReq, req);
                parts2.add(new double[]{lo, hi, Math.max(s.dn, Rules.DN[req])});
                if (Rules.DN[req] > s.dn) {
                    if (open != null && open.requiredDn == Rules.DN[req] && Math.abs(open.pts[open.pts.length - 1][0]
                            - Geo.pointAt(s.pts, lo)[0]) < 1e-6 && Math.abs(open.pts[open.pts.length - 1][1]
                            - Geo.pointAt(s.pts, lo)[1]) < 1e-6) {
                        // Соседняя часть того же требуемого диаметра — продлеваем ту же реконструируемую линию.
                        open.pts = Geo.join(open.pts, Geo.sub(s.pts, lo, hi));
                        open.length = Geo.length(open.pts);
                        open.addedFlow = Math.max(open.addedFlow, add);
                        open.cost = open.length * Rules.COST_RECON[req];
                    } else {
                        Solution.Recon r = new Solution.Recon();
                        r.segment = s;
                        r.pts = Geo.sub(s.pts, lo, hi);
                        r.existingFlow = s.flowEff;
                        r.addedFlow = add;
                        r.existingDn = s.dn;
                        r.requiredDn = Rules.DN[req];
                        r.length = Geo.length(r.pts);
                        r.cost = r.length * Rules.COST_RECON[req];
                        sol.recons.add(r);
                        open = r;
                    }
                } else {
                    open = null;
                }
            }
            Solution.Impact im = new Solution.Impact();
            im.segment = s;
            im.addedFlow = maxAdd;
            double cap = Rules.CAPACITY[Rules.indexOfDn(s.dn)];
            im.utilizationBefore = s.flowEff / cap;
            im.utilizationAfter = (s.flowEff + maxAdd) / cap;
            im.requiredDn = Rules.DN[maxReq];
            sol.impacts.add(im);
        }
        return finalDn;
    }

    // ---- Камеры и врезки (раздел 8.2) -----------------------------------------------------------------------------

    /** Итоговый диаметр существующего участка в точке along (не меньше исходного). */
    private static int dnAt(Map<PlanModel.Segment, List<double[]>> finalDn, PlanModel.Segment s, double along) {
        int dn = s.dn;
        List<double[]> parts = finalDn.get(s);
        if (parts != null) {
            for (double[] p : parts) {
                if (along >= p[0] - 1e-6 && along <= p[1] + 1e-6) {
                    dn = Math.max(dn, (int) p[2]);
                }
            }
        }
        return dn;
    }

    private void chambersAndTieIns(Map<PlanModel.Segment, List<double[]>> finalDn) {
        for (Tree.Node n : tree.nodes) {
            if (n.kind == Tree.Kind.TERMINAL) {
                continue;
            }
            // Наибольший диаметр всех участков, примыкающих к узлу в итоговом варианте.
            int maxDn = 0;
            if (n.in != null) {
                maxDn = Rules.DN[edgeDn.get(n.in)[1]];
            }
            int childMax = 0;
            for (Tree.Edge e : n.out) {
                childMax = Math.max(childMax, Rules.DN[edgeDn.get(e)[0]]);
            }
            maxDn = Math.max(maxDn, childMax);

            if (n.kind == Tree.Kind.JUNCTION) {
                Solution.NewChamber c = new Solution.NewChamber();
                c.node = n;
                c.dn = maxDn;
                c.cost = Rules.chamberCost(maxDn);
                sol.chambers.add(c);
                continue;
            }

            Solution.TieIn tie = new Solution.TieIn();
            tie.node = n;
            tie.requiredDn = childMax;
            if (n.kind == Tree.Kind.TIE_SEGMENT) {
                PlanModel.Segment s = n.segment;
                maxDn = Math.max(maxDn, Math.max(dnAt(finalDn, s, n.along - 0.01), dnAt(finalDn, s, n.along + 0.01)));
                tie.existingId = s.id;
                tie.existingType = "heat_network";
                tie.existingDn = s.dn;
                Solution.NewChamber c = new Solution.NewChamber();
                c.node = n;
                c.dn = maxDn;
                c.cost = Rules.chamberCost(maxDn);
                sol.chambers.add(c);
            } else {
                PlanModel.Chamber ch = n.chamber;
                for (PlanModel.Segment s : ch.neighbors) {
                    // Участок, у которого камера — верхний конец (upstream = камера), примыкает своим началом; иначе — концом.
                    double along = ch.id.equals(s.upstreamId) ? 0.01 : s.length - 0.01;
                    maxDn = Math.max(maxDn, dnAt(finalDn, s, along));
                }
                tie.existingId = ch.id;
                tie.existingType = "heat_chamber";
                tie.existingDn = ch.dnEff;
                // Камера подлежит реконструкции, только если требуемый диаметр вырос выше её исходного.
                if (maxDn > ch.dnEff) {
                    Solution.ChamberRecon r = new Solution.ChamberRecon();
                    r.chamber = ch;
                    r.existingDn = ch.dnEff;
                    r.requiredDn = maxDn;
                    r.cost = Rules.chamberCost(maxDn);
                    sol.chamberRecons.add(r);
                }
            }
            sol.tieIns.add(tie);
        }
    }
}
