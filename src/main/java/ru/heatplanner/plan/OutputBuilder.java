package ru.heatplanner.plan;

import org.locationtech.jts.geom.Coordinate;
import ru.heatplanner.Utm;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Выходной GeoJSON по разделу 7 приложения: один FeatureCollection в WGS 84, варианты различаются variant_id.
 * Объекты: heat_network (участки между узлами), heat_chamber (только новые камеры), technical_node (где без разветвления
 * меняется диаметр или способ прокладки) и variant_summary (geometry = null). Начало и конец участка совпадают с узлами
 * по id, повороты остаются внутренними вершинами. Стоимости округлены до копеек, итоги считаются по округлённым
 * значениям, чтобы файл был внутренне согласован.
 */
public final class OutputBuilder {

    private OutputBuilder() {
    }

    private static double cents(double v) {
        return Math.round(v * 100) / 100.0;
    }

    private static Object idValue(Id id) {
        return id.isNumeric() ? (Object) Long.valueOf(id.value()) : id.value();
    }

    private static double[] wgs(Coordinate c) {
        double[] p = Utm.inverse(c.x, c.y);
        return new double[]{Math.round(p[0] * 1e9) / 1e9, Math.round(p[1] * 1e9) / 1e9};
    }

    private static Map<String, Object> feature(Map<String, Object> props, String type, Object coordinates) {
        Map<String, Object> f = new LinkedHashMap<>();
        f.put("type", "Feature");
        f.put("properties", props);
        if (type == null) {
            f.put("geometry", null);
        } else {
            Map<String, Object> g = new LinkedHashMap<>();
            g.put("type", type);
            g.put("coordinates", coordinates);
            f.put("geometry", g);
        }
        return f;
    }

    public static Map<String, Object> build(List<Variants.Variant> variants) {
        List<Object> features = new ArrayList<>();
        int rank = 1;
        for (Variants.Variant v : variants) {
            addVariant(features, v.solution, "v" + rank, rank);
            rank++;
        }
        Map<String, Object> fc = new LinkedHashMap<>();
        fc.put("type", "FeatureCollection");
        fc.put("features", features);
        return fc;
    }

    private static final class Segment {
        Object startNode;
        Coordinate startXy;
        final List<Coordinate> points = new ArrayList<>();
        int dn;
        boolean special;
        int lastLeg = -1;
        double flow;
        double length;
        double cost;
    }

    private static void addVariant(List<Object> out, JointPlanner.Solution s, String variantId, int rank) {
        TreeEvaluator.Result ev = s.evaluation;
        int n = s.branches.size();
        // Узел, в который заканчивается каждая ветвь (существующая или новая камера), и его координаты.
        Object[] endNode = new Object[n];
        int chamberNo = 0;
        double chamberCost = 0;
        double tieInCost = 0;
        int tieIns = 0;
        for (TreeEvaluator.Chamber c : ev.chambers) {
            if (c.existing) {
                endNode[c.branch] = idValue(s.branches.get(c.branch).existing.chamber.id);
                tieIns++;
                tieInCost += cents(c.cost);
            } else {
                String id = variantId + "_chamber_" + (++chamberNo);
                endNode[c.branch] = id;
                Map<String, Object> props = new LinkedHashMap<>();
                props.put("id", id);
                props.put("object_type", "heat_chamber");
                props.put("variant_id", variantId);
                props.put("diameter", c.dn);
                props.put("cost", cents(c.cost));
                out.add(feature(props, "Point", wgs(c.xy)));
                chamberCost += cents(c.cost);
            }
        }
        List<List<TreeEvaluator.Piece>> pieces = new ArrayList<>();
        for (int i = 0; i < n; i++) {
            pieces.add(new ArrayList<>());
        }
        for (TreeEvaluator.Piece p : ev.pieces) {
            pieces.get(p.branch).add(p);
        }
        double pipeCost = 0;
        double length = 0;
        int netNo = 0;
        int nodeNo = 0;
        for (int b = 0; b < n; b++) {
            Branch br = s.branches.get(b);
            // Места врезок дочерних ветвей: позиция вдоль ветви и узел-камера.
            List<double[]> joinPos = new ArrayList<>();
            List<Object> joinNode = new ArrayList<>();
            for (int c = 0; c < n; c++) {
                if (s.branches.get(c).parent == b) {
                    joinPos.add(new double[]{s.branches.get(c).parentPos});
                    joinNode.add(endNode[c]);
                }
            }
            Segment cur = null;
            Object nextStartNode = idValue(br.target.id);
            Coordinate nextStartXy = br.target.xy;
            double cum = 0;
            List<TreeEvaluator.Piece> ps = pieces.get(b);
            for (int i = 0; i < ps.size(); i++) {
                TreeEvaluator.Piece p = ps.get(i);
                boolean special = br.route.legs.get(p.leg).special;
                if (cur != null && (p.dn != cur.dn || special != cur.special || (special && p.leg != cur.lastLeg))) {
                    String id = variantId + "_node_" + (++nodeNo);
                    Coordinate at = cur.points.get(cur.points.size() - 1);
                    out.add(technicalNode(id, variantId, at));
                    netNo = close(out, cur, id, at, variantId, netNo);
                    nextStartNode = id;
                    nextStartXy = at;
                    cur = null;
                }
                if (cur == null) {
                    cur = new Segment();
                    cur.startNode = nextStartNode;
                    cur.startXy = nextStartXy;
                    cur.points.add(nextStartXy);
                    cur.dn = p.dn;
                    cur.special = special;
                    cur.flow = p.flow;
                }
                cur.points.add(p.to);
                cur.lastLeg = p.leg;
                cur.length += p.length;
                cur.cost += p.cost;
                cum += p.length;
                Object node = null;
                for (int j = 0; j < joinPos.size(); j++) {
                    if (Math.abs(joinPos.get(j)[0] - cum) < 1e-6) {
                        node = joinNode.get(j);
                    }
                }
                if (node != null) {
                    netNo = close(out, cur, node, p.to, variantId, netNo);
                    nextStartNode = node;
                    nextStartXy = p.to;
                    cur = null;
                }
            }
            if (cur != null) {
                netNo = close(out, cur, endNode[b], br.attach, variantId, netNo);
            }
        }
        for (Object f : out) {
            // Итоги считаются по значениям, записанным в файл.
            Map<String, Object> props = (Map<String, Object>) ((Map<String, Object>) f).get("properties");
            if ("heat_network".equals(props.get("object_type")) && variantId.equals(props.get("variant_id"))) {
                pipeCost += (Double) props.get("cost");
                length += (Double) props.get("length");
            }
        }
        double construction = cents(pipeCost + chamberCost + tieInCost);
        double penalty = cents(s.penalty);
        double calculated = cents(construction + penalty);
        Map<String, Object> props = new LinkedHashMap<>();
        props.put("id", variantId + "_summary");
        props.put("object_type", "variant_summary");
        props.put("variant_id", variantId);
        props.put("rank", rank);
        props.put("construction_cost", construction);
        props.put("chamber_construction_cost", cents(chamberCost));
        props.put("existing_chamber_tie_in_count", tieIns);
        props.put("existing_chamber_tie_in_cost", cents(tieInCost));
        props.put("unconnected_penalty", penalty);
        props.put("calculated_cost", calculated);
        props.put("new_network_length", cents(length));
        props.put("score", Math.round(Rules.score(calculated, length) * 10000) / 10000.0);
        List<Object> ids = new ArrayList<>();
        for (JointPlanner.Unconnected u : s.unconnected) {
            ids.add(idValue(u.target.id));
        }
        props.put("unconnected_oks_ids", ids);
        out.add(feature(props, null, null));
    }

    private static Map<String, Object> technicalNode(String id, String variantId, Coordinate at) {
        Map<String, Object> props = new LinkedHashMap<>();
        props.put("id", id);
        props.put("object_type", "technical_node");
        props.put("variant_id", variantId);
        return feature(props, "Point", wgs(at));
    }

    /** Записывает участок от начального узла до конечного и возвращает новый счётчик участков. */
    private static int close(List<Object> out, Segment seg, Object endNode, Coordinate endXy, String variantId, int netNo) {
        seg.points.set(seg.points.size() - 1, endXy);
        List<double[]> coords = new ArrayList<>();
        for (Coordinate c : seg.points) {
            coords.add(wgs(c));
        }
        Map<String, Object> props = new LinkedHashMap<>();
        props.put("id", variantId + "_net_" + (netNo + 1));
        props.put("object_type", "heat_network");
        props.put("variant_id", variantId);
        props.put("start_node_id", seg.startNode);
        props.put("end_node_id", endNode);
        props.put("flow_tph", Math.round(seg.flow * 100) / 100.0);
        props.put("diameter", seg.dn);
        props.put("length", cents(seg.length));
        props.put("laying_method", seg.special ? "special" : "base");
        props.put("depth_start", null);
        props.put("depth_end", null);
        props.put("cost", cents(seg.cost));
        out.add(feature(props, "LineString", coords));
        return netNo + 1;
    }
}