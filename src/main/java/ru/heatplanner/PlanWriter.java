package ru.heatplanner;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Формирует ответ сервиса: один GeoJSON со всеми вариантами в формате раздела 10 технического приложения
 * и «meta» — то, чего в ТЗ нет, но что нужно для объяснения результата: разбивка стоимости, причины неподключения,
 * качество трасс, влияние на существующую сеть, экономия от совместного подключения, устойчивость рейтинга, правила.
 */
final class PlanWriter {

    private static final ObjectMapper MAPPER = new ObjectMapper();

    private PlanWriter() {
    }

    static ObjectNode write(Planner.Result r) {
        ObjectNode root = MAPPER.createObjectNode();
        ArrayNode features = MAPPER.createArrayNode();
        ArrayNode variants = MAPPER.createArrayNode();
        double separateCost = r.separate.solution.totalCost();
        for (Planner.Variant v : r.variants) {
            features.addAll(variantFeatures(v));
            variants.add(variantMeta(v, r, separateCost));
        }

        ObjectNode geojson = MAPPER.createObjectNode();
        geojson.put("type", "FeatureCollection");
        geojson.set("features", features);
        root.set("geojson", geojson);

        ObjectNode meta = MAPPER.createObjectNode();
        meta.put("cell_m", round(r.map.cell, 2));
        meta.put("oks_total", r.model.targets.size());
        meta.put("existing_segments", r.model.segments.size());
        meta.put("existing_chambers", r.model.chambers.size());
        meta.set("assumptions", strings(r.model.assumptions));
        List<String> diagnostics = new ArrayList<>(r.model.diagnostics);
        diagnostics.addAll(clearanceNotes(r));
        meta.set("diagnostics", strings(diagnostics));
        meta.set("notes", strings(r.notes));
        meta.set("variants", variants);
        meta.set("joint", jointMeta(r));
        meta.set("sensitivity", sensitivity(r));
        meta.set("rules", rules());
        root.set("meta", meta);
        return root;
    }

    // ---- Объекты выходного GeoJSON (раздел 10) --------------------------------------------------------------------

    private static List<ObjectNode> variantFeatures(Planner.Variant v) {
        String vid = String.valueOf(v.rank);
        Solution s = v.solution;
        Map<Object, String> ids = new IdentityHashMap<>();
        List<ObjectNode> out = new ArrayList<>();

        // Идентификаторы узлов заранее: участки ссылаются на них по start_node_id / end_node_id.
        int tieNo = 0;
        for (Solution.TieIn t : s.tieIns) {
            ids.put(t.node, "tie_" + vid + "_" + (++tieNo));
        }
        int chamberNo = 0;
        for (Solution.NewChamber c : s.chambers) {
            if (c.node.kind == Tree.Kind.JUNCTION) {
                ids.put(c.node, "chamber_" + vid + "_" + (++chamberNo));
            }
        }
        for (Tree.Node n : v.tree.nodes) {
            if (n.kind == Tree.Kind.TERMINAL) {
                ids.put(n, n.point.id);
            }
        }
        int techNo = 0;
        for (Solution.TechNode t : s.techNodes) {
            ids.put(t, "node_" + vid + "_" + (++techNo));
        }

        int pieceNo = 0;
        for (Solution.Piece p : s.pieces) {
            ObjectNode props = MAPPER.createObjectNode();
            props.put("id", "new_" + vid + "_" + (++pieceNo));
            props.put("object_type", "heat_network");
            props.put("variant_id", vid);
            props.put("start_node_id", ids.get(p.start));
            props.put("end_node_id", ids.get(p.end));
            props.put("flow_tph", round(p.flow, 3));
            props.put("diameter", p.dn);
            props.put("length", round(p.length, 2));
            props.put("laying_method", p.special ? "special" : "base");
            props.putNull("depth_start");
            props.putNull("depth_end");
            props.put("cost", round(p.cost, 2));
            out.add(feature(line(p.pts), props));
        }
        for (Solution.TieIn t : s.tieIns) {
            ObjectNode props = MAPPER.createObjectNode();
            props.put("id", ids.get(t.node));
            props.put("object_type", "tie_in");
            props.put("variant_id", vid);
            props.put("existing_object_id", t.existingId);
            props.put("existing_object_type", t.existingType);
            props.put("existing_diameter", t.existingDn);
            props.put("required_diameter", t.requiredDn);
            props.put("cost", round(t.cost, 2));
            out.add(feature(point(t.node.xy), props));
        }
        int reconNo = 0;
        for (Solution.Recon rc : s.recons) {
            ObjectNode props = MAPPER.createObjectNode();
            props.put("id", "recon_" + vid + "_" + (++reconNo));
            props.put("object_type", "heat_network_reconstruction");
            props.put("variant_id", vid);
            props.put("existing_object_id", rc.segment.id);
            props.put("existing_flow_tph", round(rc.existingFlow, 3));
            props.put("added_flow_tph", round(rc.addedFlow, 3));
            props.put("calculated_flow_tph", round(rc.existingFlow + rc.addedFlow, 3));
            props.put("existing_diameter", rc.existingDn);
            props.put("required_diameter", rc.requiredDn);
            props.put("length", round(rc.length, 2));
            props.put("cost", round(rc.cost, 2));
            out.add(feature(line(rc.pts), props));
        }
        int newChamberNo = 0;
        for (Solution.NewChamber c : s.chambers) {
            ObjectNode props = MAPPER.createObjectNode();
            // Камеры-развилки называются так же, как на них ссылаются участки; камера в точке врезки — отдельный номер.
            props.put("id", c.node.kind == Tree.Kind.JUNCTION ? ids.get(c.node)
                    : "chamber_" + vid + "_t" + (++newChamberNo));
            props.put("object_type", "heat_chamber");
            props.put("variant_id", vid);
            props.put("diameter", c.dn);
            props.put("cost", round(c.cost, 2));
            out.add(feature(point(c.node.xy), props));
        }
        int chamberReconNo = 0;
        for (Solution.ChamberRecon rc : s.chamberRecons) {
            ObjectNode props = MAPPER.createObjectNode();
            props.put("id", "chamber_recon_" + vid + "_" + (++chamberReconNo));
            props.put("object_type", "heat_chamber_reconstruction");
            props.put("variant_id", vid);
            props.put("existing_object_id", rc.chamber.id);
            props.put("existing_diameter", rc.existingDn);
            props.put("required_diameter", rc.requiredDn);
            props.put("cost", round(rc.cost, 2));
            out.add(feature(point(rc.chamber.xy), props));
        }
        for (Solution.TechNode t : s.techNodes) {
            ObjectNode props = MAPPER.createObjectNode();
            props.put("id", ids.get(t));
            props.put("object_type", "technical_node");
            props.put("variant_id", vid);
            out.add(feature(point(t.xy), props));
        }

        ObjectNode sum = MAPPER.createObjectNode();
        sum.put("id", "summary_" + vid);
        sum.put("object_type", "variant_summary");
        sum.put("variant_id", vid);
        sum.put("rank", v.rank);
        sum.put("construction_cost", round(s.constructionCost, 2));
        sum.put("chamber_construction_cost", round(s.chamberCost, 2));
        sum.put("tie_in_cost", round(s.tieInCost, 2));
        sum.put("reconstruction_cost", round(s.reconCost, 2));
        sum.put("chamber_reconstruction_cost", round(s.chamberReconCost, 2));
        sum.put("unconnected_penalty", round(s.penalty, 2));
        sum.put("calculated_cost", round(s.totalCost(), 2));
        sum.put("new_network_length", round(s.newLength, 2));
        sum.put("reconstruction_length", round(s.reconLength, 2));
        sum.put("length", round(s.totalLength(), 2));
        sum.put("score", round(s.score(), 4));
        ArrayNode unconnected = sum.putArray("unconnected_oks_ids");
        s.unconnected.forEach(t -> unconnected.add(t.key));
        ObjectNode f = MAPPER.createObjectNode();
        f.put("type", "Feature");
        f.putNull("geometry");
        f.set("properties", sum);
        out.add(f);
        return out;
    }

    // ---- meta: объяснения, которых нет в ТЗ ---------------------------------------------------------------------

    private static ObjectNode variantMeta(Planner.Variant v, Planner.Result r, double separateCost) {
        Solution s = v.solution;
        ObjectNode m = MAPPER.createObjectNode();
        m.put("variant_id", String.valueOf(v.rank));
        m.put("rank", v.rank);
        m.put("profile", v.profile.id);
        m.put("name", v.profile.name);
        m.put("idea", v.profile.idea);
        m.put("score", round(s.score(), 4));
        m.put("calculated_cost", round(s.totalCost(), 2));
        ObjectNode breakdown = m.putObject("breakdown");
        breakdown.put("construction_cost", round(s.constructionCost, 2));
        breakdown.put("chamber_construction_cost", round(s.chamberCost, 2));
        breakdown.put("tie_in_cost", round(s.tieInCost, 2));
        breakdown.put("reconstruction_cost", round(s.reconCost, 2));
        breakdown.put("chamber_reconstruction_cost", round(s.chamberReconCost, 2));
        breakdown.put("unconnected_penalty", round(s.penalty, 2));
        m.put("new_network_length", round(s.newLength, 2));
        m.put("reconstruction_length", round(s.reconLength, 2));
        m.put("length", round(s.totalLength(), 2));
        m.put("connected_oks", r.model.targets.size() - s.unconnected.size());
        m.put("tie_in_count", s.tieIns.size());
        m.put("new_chamber_count", s.chambers.size());
        m.put("technical_node_count", s.techNodes.size());
        m.put("special_length", round(s.pieces.stream().filter(p -> p.special).mapToDouble(p -> p.length).sum(), 2));

        // Совместные трубы: сколько метров несут расход нескольких ОКС сразу.
        Map<Tree.Node, Integer> count = new IdentityHashMap<>();
        for (Tree.Node n : v.tree.nodes) {
            if (n.isTie()) {
                countTerminals(n, count);
            }
        }
        double shared = 0;
        for (Tree.Edge e : v.tree.edges) {
            if (count.get(e.to) > 1) {
                shared += e.length;
            }
        }
        m.put("shared_length", round(shared, 2));

        ArrayNode unconnected = m.putArray("unconnected");
        for (PlanModel.Target t : s.unconnected) {
            ObjectNode u = unconnected.addObject();
            u.put("oks_id", t.key);
            u.put("flow_tph", round(t.flow, 3));
            u.put("reason", v.reasons.getOrDefault(t, "причина не определена"));
            if (!t.points.isEmpty()) {
                u.put("point_id", t.points.get(0).id);
                u.set("position", coordinate(t.points.get(0).xy));
            }
        }

        // Качество трасс: длина пути, извилистость и число поворотов для каждого ОКС.
        Map<Tree.Node, Integer> lastDn = new IdentityHashMap<>();
        for (Solution.Piece p : s.pieces) {
            if (p.end instanceof Tree.Node) {
                lastDn.put((Tree.Node) p.end, p.dn);
            }
        }
        ArrayNode routes = m.putArray("routes");
        for (Tree.Node leaf : v.tree.terminals()) {
            double len = 0;
            int turns = 0;
            Tree.Node root = leaf;
            while (root.in != null) {
                len += root.in.length;
                turns += Geo.turns(root.in.pts, 25);
                root = root.in.from;
            }
            double straight = Geo.dist(root.xy, leaf.xy);
            ObjectNode route = routes.addObject();
            route.put("oks_id", leaf.target.key);
            route.put("point_id", leaf.point.id);
            route.put("flow_tph", round(leaf.target.flow, 3));
            route.put("length", round(len, 1));
            route.put("straight", round(straight, 1));
            route.put("detour_ratio", straight > 1 ? round(len / straight, 2) : 1.0);
            route.put("turns", turns);
            route.put("diameter", lastDn.getOrDefault(leaf, 0));
        }

        ArrayNode impacts = m.putArray("impacts");
        for (Solution.Impact im : s.impacts) {
            ObjectNode i = impacts.addObject();
            i.put("segment_id", im.segment.id);
            i.put("added_flow_tph", round(im.addedFlow, 3));
            i.put("utilization_before", round(im.utilizationBefore, 3));
            i.put("utilization_after", round(im.utilizationAfter, 3));
            i.put("diameter", im.segment.dn);
            i.put("required_diameter", im.requiredDn);
        }

        List<String> checks = new ArrayList<>();
        checks.addAll(angleChecks(v));
        checks.addAll(clearanceChecks(v, r));
        m.set("checks", strings(checks));

        m.set("explanation", strings(explain(v, r, separateCost)));
        return m;
    }

    private static int countTerminals(Tree.Node n, Map<Tree.Node, Integer> count) {
        int c = 0;
        if (n.kind == Tree.Kind.TERMINAL) {
            c = 1;
        } else {
            for (Tree.Edge e : n.out) {
                c += countTerminals(e.to, count);
            }
        }
        count.put(n, c);
        return c;
    }

    /** Углы пересечения дорог и трамвайных путей: по таблице 5.1 не менее 45°. */
    private static List<String> angleChecks(Planner.Variant v) {
        List<String> out = new ArrayList<>();
        for (Tree.Edge e : v.tree.edges) {
            for (Zones.Zone z : e.zones) {
                if (z.rule.minAngleDeg > 0 && z.angleDeg < z.rule.minAngleDeg - 1e-6) {
                    out.add(String.format(Locale.ROOT, "Пересечение «%s»: угол %.0f° меньше допустимых %.0f°.",
                            z.rule.title, z.angleDeg, z.rule.minAngleDeg));
                }
            }
        }
        return out;
    }

    /** Зазор до существующих ОКС растёт с диаметром (5/7/9 м): проверяем итоговые диаметры труб. */
    private static List<String> clearanceChecks(Planner.Variant v, Planner.Result r) {
        List<String> out = new ArrayList<>();
        for (Solution.Piece p : v.solution.pieces) {
            double need = Rules.oksDistance(p.dn);
            if (need <= 5.0) {
                continue;
            }
            for (PlanModel.Shape s : r.model.shapes) {
                if (!"oks".equals(s.rule.type)) {
                    continue;
                }
                for (int i = 1; i < p.pts.length; i++) {
                    for (double[][] ring : s.rings) {
                        for (int j = 1; j < ring.length; j++) {
                            if (Geo.segmentSegment(p.pts[i - 1], p.pts[i], ring[j - 1], ring[j]) < need - 1e-6) {
                                out.add("Труба DN " + p.dn + " проходит ближе " + need + " м к существующему ОКС "
                                        + s.id + ".");
                                i = p.pts.length;
                                j = ring.length;
                                break;
                            }
                        }
                    }
                }
            }
        }
        return out;
    }

    /** Заметки о точках подключения, стоящих ближе положенного зазора к запретным зонам (зазор у точки не применяется). */
    private static List<String> clearanceNotes(Planner.Result r) {
        List<String> out = new ArrayList<>();
        int insideOwn = 0;
        for (PlanModel.Target t : r.model.targets) {
            for (PlanModel.ConnectionPoint cp : t.points) {
                for (PlanModel.Shape s : r.model.shapes) {
                    if (!s.rule.forbidden || out.size() >= 10) {
                        continue;
                    }
                    double d = s.rule.minDistance;
                    if (cp.xy[0] < s.minX - d || cp.xy[0] > s.maxX + d || cp.xy[1] < s.minY - d || cp.xy[1] > s.maxY + d) {
                        continue;
                    }
                    boolean in = s.area && Geo.inside(cp.xy[0], cp.xy[1], s.rings);
                    double boundary = Geo.pointBoundary(cp.xy[0], cp.xy[1], s.rings);
                    if (in && "oks".equals(s.rule.type)) {
                        insideOwn++;
                    } else if (in || boundary < d) {
                        out.add(String.format(Locale.ROOT, "Точка подключения %s стоит в %.1f м от объекта «%s» (%s), "
                                        + "минимум %.1f м: у самой точки зазор не применялся.",
                                cp.id, in ? 0.0 : boundary, s.id, s.rule.title, d));
                    }
                }
            }
        }
        if (insideOwn > 0) {
            out.add(0, "У " + insideOwn + " точек подключения точка лежит внутри контура существующего ОКС "
                    + "(restriction_type = oks). Контур принят зданием самого ОКС: труба заканчивается у его стены и под здание не заходит, "
                    + "зазор 5 м до этого здания не применяется. К чужим зданиям зазор действует.");
        }
        return out;
    }

    private static List<String> explain(Planner.Variant v, Planner.Result r, double separateCost) {
        Solution s = v.solution;
        List<String> out = new ArrayList<>();
        int total = r.model.targets.size();
        long inChambers = s.tieIns.stream().filter(t -> "heat_chamber".equals(t.existingType)).count();
        out.add(String.format(Locale.ROOT, "Подключено %d из %d ОКС: %d врезок (в существующие камеры: %d), новых камер %d.",
                total - s.unconnected.size(), total, s.tieIns.size(), inChambers, s.chambers.size()));
        out.add(String.format(Locale.ROOT, "Новая сеть %.0f м, реконструкция существующей сети %.0f м; стоимость %.1f млн руб., "
                + "показатель S = %.3f.", s.newLength, s.reconLength, s.totalCost() / 1e6, s.score()));
        if (!s.recons.isEmpty()) {
            out.add("Реконструкция нужна на участках существующей сети: "
                    + s.recons.stream().map(x -> x.segment.id + " (DN " + x.existingDn + " → " + x.requiredDn + ")")
                    .distinct().limit(6).collect(java.util.stream.Collectors.joining(", ")) + ".");
        } else {
            out.add("Существующая сеть выдерживает добавленный расход: реконструкция не требуется.");
        }
        Planner.Variant best = r.variants.get(0);
        if (v != best) {
            Solution b = best.solution;
            out.add(String.format(Locale.ROOT, "По сравнению с лучшим вариантом: стоимость %s%.1f млн руб., длина %s%.0f м.",
                    s.totalCost() >= b.totalCost() ? "+" : "−", Math.abs(s.totalCost() - b.totalCost()) / 1e6,
                    s.totalLength() >= b.totalLength() ? "+" : "−", Math.abs(s.totalLength() - b.totalLength())));
        }
        if (!s.unconnected.isEmpty()) {
            out.add("Не подключено ОКС: " + s.unconnected.size() + ". Штраф включён в стоимость.");
        }
        out.add(v.profile.idea);
        return out;
    }

    private static ObjectNode jointMeta(Planner.Result r) {
        ObjectNode j = MAPPER.createObjectNode();
        Solution sep = r.separate.solution;
        Solution best = r.variants.get(0).solution;
        j.put("separate_cost", round(sep.totalCost(), 2));
        j.put("separate_length", round(sep.totalLength(), 2));
        j.put("separate_tie_ins", sep.tieIns.size());
        j.put("best_cost", round(best.totalCost(), 2));
        double saving = sep.totalCost() - best.totalCost();
        j.put("saving_rub", round(saving, 2));
        j.put("saving_pct", sep.totalCost() > 0 ? round(100 * saving / sep.totalCost(), 1) : 0);
        j.put("comment", "База для сравнения: тот же расчёт, но каждый ОКС подключается отдельно, без совместных труб.");
        return j;
    }

    /** Как меняется лучший вариант, если менять вес стоимости в рейтинге: устойчив ли выбор. */
    private static ObjectNode sensitivity(Planner.Result r) {
        ObjectNode out = MAPPER.createObjectNode();
        ArrayNode rows = out.putArray("weights");
        String first = null;
        boolean stable = true;
        for (int w = 50; w <= 90; w += 5) {
            double wc = w / 100.0;
            Planner.Variant winner = null;
            double bestValue = Double.MAX_VALUE;
            for (Planner.Variant v : r.variants) {
                double value = wc * v.solution.totalCost() / Rules.COST_SCALE
                        + (1 - wc) * v.solution.totalLength() / Rules.LENGTH_SCALE;
                if (value < bestValue) {
                    bestValue = value;
                    winner = v;
                }
            }
            ObjectNode row = rows.addObject();
            row.put("weight_cost", wc);
            row.put("winner", String.valueOf(winner.rank));
            if (first == null) {
                first = String.valueOf(winner.rank);
            } else if (!first.equals(String.valueOf(winner.rank))) {
                stable = false;
            }
        }
        out.put("stable", stable);
        out.put("comment", stable
                ? "Лучший вариант остаётся лучшим при весе стоимости от 50 % до 90 %: выбор не зависит от спорных весов."
                : "Лучший вариант меняется в зависимости от веса стоимости: выбор зависит от приоритета заказчика.");
        return out;
    }

    /** Правила расчёта как данные: их же показывает экран «Как это посчитано». */
    static ObjectNode rules() {
        ObjectNode out = MAPPER.createObjectNode();
        ArrayNode dn = out.putArray("diameters");
        for (int i = 0; i < Rules.DN.length; i++) {
            ObjectNode d = dn.addObject();
            d.put("dn", Rules.DN[i]);
            d.put("capacity_tph", Rules.CAPACITY[i]);
            d.put("max_length_m", Rules.MAX_LENGTH[i]);
            d.put("cost_new", Rules.COST_NEW[i]);
            d.put("cost_reconstruction", Rules.COST_RECON[i]);
        }
        ArrayNode rs = out.putArray("restrictions");
        for (Rules.Rule rule : Rules.RULES.values()) {
            ObjectNode o = rs.addObject();
            o.put("type", rule.type);
            o.put("title", rule.title);
            o.put("forbidden", rule.forbidden);
            o.put("min_distance_m", rule.minDistance);
            o.put("k_special", rule.kSpec);
            o.put("special_extent_m", rule.extent);
            o.put("min_angle_deg", rule.minAngleDeg);
        }
        out.put("tie_in_cost", Rules.TIE_IN_COST);
        out.put("chamber_snap_m", Rules.CHAMBER_SNAP_M);
        out.put("max_chamber_segments", Rules.MAX_CHAMBER_SEGMENTS);
        out.put("penalty_fixed", Rules.PENALTY_FIXED);
        out.put("penalty_per_tph", Rules.PENALTY_PER_TPH);
        out.put("weight_cost", Rules.WEIGHT_COST);
        out.put("weight_length", Rules.WEIGHT_LENGTH);
        out.put("cost_scale", Rules.COST_SCALE);
        out.put("length_scale", Rules.LENGTH_SCALE);
        return out;
    }

    // ---- Вспомогательное --------------------------------------------------------------------------------------------

    private static ObjectNode feature(ObjectNode geometry, ObjectNode props) {
        ObjectNode f = MAPPER.createObjectNode();
        f.put("type", "Feature");
        f.set("geometry", geometry);
        f.set("properties", props);
        return f;
    }

    private static ObjectNode point(double[] xy) {
        ObjectNode g = MAPPER.createObjectNode();
        g.put("type", "Point");
        g.set("coordinates", coordinate(xy));
        return g;
    }

    private static ObjectNode line(double[][] pts) {
        ObjectNode g = MAPPER.createObjectNode();
        g.put("type", "LineString");
        ArrayNode coords = g.putArray("coordinates");
        for (double[] p : pts) {
            coords.add(coordinate(p));
        }
        return g;
    }

    /** UTM → градусы WGS 84, семь знаков после запятой (около 1 см). */
    private static ArrayNode coordinate(double[] xy) {
        double[] ll = Utm.inverse(xy[0], xy[1]);
        ArrayNode c = MAPPER.createArrayNode();
        c.add(Math.round(ll[0] * 1e7) / 1e7);
        c.add(Math.round(ll[1] * 1e7) / 1e7);
        return c;
    }

    private static ArrayNode strings(List<String> list) {
        ArrayNode a = MAPPER.createArrayNode();
        list.forEach(a::add);
        return a;
    }

    private static double round(double v, int digits) {
        double f = Math.pow(10, digits);
        return Math.round(v * f) / f;
    }
}
