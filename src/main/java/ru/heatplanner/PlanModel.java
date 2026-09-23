package ru.heatplanner;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Входные данные расчёта в метрах (EPSG:32637): существующая сеть, ОКС, ограничения.
 *
 * Файл уже проверен GeoJsonInspector, поэтому здесь чтение снисходительное. Всё, чего в данных не хватает
 * (расходы участков, диаметры камер, oks_id), не считается ошибкой: подставляется допущение и записывается
 * в assumptions, чтобы пользователь видел, на чём построен расчёт.
 */
final class PlanModel {

    /** Существующий участок. pts[0] — конец, ближайший к источнику (после ориентации). */
    static final class Segment {
        String id;
        int dn;
        Double flow;
        /** Расход для расчёта: из файла или допущение. */
        double flowEff;
        String upstreamId;
        double[][] pts;
        double length;
        /** Ближайший к источнику существующий участок выше по цепочке (камеры пропускаются); null — дальше источник. */
        Segment upSeg;
    }

    static final class Chamber {
        String id;
        Integer dn;
        int dnEff;
        double[] xy;
        String upstreamId;
        Segment upSeg;
        /** Сколько участков примыкает к камере сейчас (к источнику и от него). */
        int adjacent;
        /** Примыкающие существующие участки. */
        final List<Segment> neighbors = new ArrayList<>();
    }

    static final class Source {
        String id;
        double[] xy;
    }

    static final class ConnectionPoint {
        String id;
        double[] xy;
    }

    /** Один перспективный ОКС: расход и одна или несколько точек, через любую из которых можно подключиться. */
    static final class Target {
        String key;
        double flow;
        List<ConnectionPoint> points = new ArrayList<>();
        /** Причина, по которой ОКС нельзя даже пытаться подключить (нет расхода или точки). */
        String problem;
    }

    /** Ограничение: полигон (кольца по правилу чётности) или линия. */
    static final class Shape {
        Rules.Rule rule;
        String id;
        boolean area;
        double[][][] rings;
        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
        double maxY = -Double.MAX_VALUE;
    }

    final Map<String, Source> sources = new LinkedHashMap<>();
    final Map<String, Segment> segments = new LinkedHashMap<>();
    final Map<String, Chamber> chambers = new LinkedHashMap<>();
    final List<Target> targets = new ArrayList<>();
    final List<Shape> shapes = new ArrayList<>();
    /** Замечания к данным (диагностика). */
    final List<String> diagnostics = new ArrayList<>();
    /** Допущения, принятые из-за отсутствующих в файле атрибутов. */
    final List<String> assumptions = new ArrayList<>();

    private static class Future {
        Double flow;
        Double heatLoad;
    }

    private static class RawPoint {
        String id;
        String oksId;
        Double flow;
        double[] xy;
    }

    private final Map<String, Future> futures = new LinkedHashMap<>();
    private final List<RawPoint> rawPoints = new ArrayList<>();
    private final Set<String> unknownTypes = new LinkedHashSet<>();
    private int droppedNetwork;

    static PlanModel parse(Path path, Map<String, String> links) throws IOException {
        PlanModel model = new PlanModel();
        ObjectMapper mapper = new ObjectMapper();
        try (JsonParser parser = mapper.getFactory().createParser(path.toFile())) {
            parser.nextToken();
            while (parser.nextToken() == JsonToken.FIELD_NAME) {
                String field = parser.currentName();
                parser.nextToken();
                if ("features".equals(field)) {
                    while (parser.nextToken() == JsonToken.START_OBJECT) {
                        model.read(mapper.readTree(parser), links);
                    }
                } else {
                    parser.skipChildren();
                }
            }
        }
        model.finish(links);
        return model;
    }

    // ---- Чтение объектов ------------------------------------------------------------------------------------------

    private void read(JsonNode feature, Map<String, String> links) {
        JsonNode props = feature.path("properties");
        JsonNode geometry = feature.path("geometry");
        String id = props.path("id").asText();
        String type = props.path("object_type").asText();
        switch (type) {
            case "source": {
                Source s = new Source();
                s.id = id;
                s.xy = project(geometry.path("coordinates"));
                sources.put(id, s);
                break;
            }
            case "heat_network": {
                if (!links.containsKey(id)) {
                    droppedNetwork++;
                    break;
                }
                Segment s = new Segment();
                s.id = id;
                s.dn = props.path("diameter").asInt();
                s.flow = props.path("flow_tph").isNumber() ? props.path("flow_tph").asDouble() : null;
                s.upstreamId = links.get(id);
                s.pts = line(geometry.path("coordinates"));
                s.length = Geo.length(s.pts);
                segments.put(id, s);
                break;
            }
            case "heat_chamber": {
                if (!links.containsKey(id)) {
                    droppedNetwork++;
                    break;
                }
                Chamber c = new Chamber();
                c.id = id;
                c.dn = props.path("diameter").isIntegralNumber() ? props.path("diameter").asInt() : null;
                c.xy = project(geometry.path("coordinates"));
                c.upstreamId = links.get(id);
                chambers.put(id, c);
                break;
            }
            case "oks_future": {
                Future f = new Future();
                f.flow = props.path("flow_tph").isNumber() ? props.path("flow_tph").asDouble() : null;
                f.heatLoad = props.path("heat_load").isNumber() ? props.path("heat_load").asDouble() : null;
                futures.put(id, f);
                break;
            }
            case "oks_connection_point": {
                RawPoint p = new RawPoint();
                p.id = id;
                p.oksId = props.path("oks_id").isMissingNode() || props.path("oks_id").isNull()
                        ? null : props.path("oks_id").asText();
                p.flow = props.path("flow_tph").isNumber() ? props.path("flow_tph").asDouble() : null;
                p.xy = project(geometry.path("coordinates"));
                rawPoints.add(p);
                break;
            }
            case "oks_existing":
                addShapes(geometry, Rules.forType("oks"), id);
                break;
            case "restriction": {
                String rt = props.path("restriction_type").asText();
                Rules.Rule rule = Rules.forType(rt);
                if (!rule.known) {
                    unknownTypes.add(rt);
                }
                addShapes(geometry, rule, id);
                break;
            }
            default:
                break;
        }
    }

    private static double[] project(JsonNode c) {
        return Utm.forward(c.get(0).asDouble(), c.get(1).asDouble());
    }

    private static double[][] line(JsonNode coords) {
        double[][] pts = new double[coords.size()][];
        for (int i = 0; i < pts.length; i++) {
            pts[i] = project(coords.get(i));
        }
        return pts;
    }

    private static double[][][] rings(JsonNode polygon) {
        double[][][] rings = new double[polygon.size()][][];
        for (int i = 0; i < rings.length; i++) {
            rings[i] = line(polygon.get(i));
        }
        return rings;
    }

    /** Разбирает геометрию ограничения в Shape: полигоны и мультиполигоны — одна область, линии и точки — линейные. */
    private void addShapes(JsonNode geometry, Rules.Rule rule, String id) {
        String type = geometry.path("type").asText();
        JsonNode c = geometry.path("coordinates");
        switch (type) {
            case "Polygon":
                addShape(rule, id, true, rings(c));
                break;
            case "MultiPolygon": {
                List<double[][]> all = new ArrayList<>();
                for (JsonNode polygon : c) {
                    for (double[][] ring : rings(polygon)) {
                        all.add(ring);
                    }
                }
                addShape(rule, id, true, all.toArray(new double[0][][]));
                break;
            }
            case "LineString":
                addShape(rule, id, false, new double[][][]{line(c)});
                break;
            case "MultiLineString": {
                double[][][] lines = new double[c.size()][][];
                for (int i = 0; i < lines.length; i++) {
                    lines[i] = line(c.get(i));
                }
                addShape(rule, id, false, lines);
                break;
            }
            case "Point": {
                double[] p = project(c);
                addShape(rule, id, false, new double[][][]{{p, p}});
                break;
            }
            case "MultiPoint": {
                double[][][] pts = new double[c.size()][][];
                for (int i = 0; i < pts.length; i++) {
                    double[] p = project(c.get(i));
                    pts[i] = new double[][]{p, p};
                }
                addShape(rule, id, false, pts);
                break;
            }
            case "GeometryCollection":
                for (JsonNode g : geometry.path("geometries")) {
                    addShapes(g, rule, id);
                }
                break;
            default:
                break;
        }
    }

    private void addShape(Rules.Rule rule, String id, boolean area, double[][][] rings) {
        Shape s = new Shape();
        s.rule = rule;
        s.id = id;
        s.area = area;
        s.rings = rings;
        for (double[][] ring : rings) {
            for (double[] p : ring) {
                s.minX = Math.min(s.minX, p[0]);
                s.minY = Math.min(s.minY, p[1]);
                s.maxX = Math.max(s.maxX, p[0]);
                s.maxY = Math.max(s.maxY, p[1]);
            }
        }
        shapes.add(s);
    }

    // ---- Сборка: ориентация участков, цепочки к источнику, расходы, ОКС -----------------------------------------------

    private void finish(Map<String, String> links) {
        if (droppedNetwork > 0) {
            diagnostics.add("Не связаны с источником и исключены из расчёта: " + droppedNetwork + " объектов сети.");
        }
        for (String type : unknownTypes) {
            diagnostics.add("Тип ограничения «" + type + "» не описан в таблице 5.1: принято правило "
                    + "«пересечение запрещено, зазор 1,0 м».");
        }

        // Ориентация: pts[0] должен быть концом, ближайшим к вышестоящему объекту (к источнику).
        for (Segment s : segments.values()) {
            double[] up = upstreamAnchor(s);
            if (up != null && Geo.dist(s.pts[s.pts.length - 1], up) < Geo.dist(s.pts[0], up)) {
                s.pts = Geo.reverse(s.pts);
            }
        }
        // Существующая сеть — тоже препятствие: новая труба может её пересечь только специальным проходом (таблица 5.1).
        Rules.Rule networkRule = Rules.RULES.get("heat_network");
        for (Segment s : segments.values()) {
            addShape(networkRule, s.id, false, new double[][][]{s.pts});
        }
        for (Segment s : segments.values()) {
            s.upSeg = nextSegment(s.upstreamId);
        }
        for (Chamber c : chambers.values()) {
            c.upSeg = nextSegment(c.upstreamId);
        }

        // Расход существующих участков: из файла; если его нет — допущение «половина пропускной способности».
        int assumedFlows = 0;
        for (Segment s : segments.values()) {
            if (s.flow != null) {
                s.flowEff = s.flow;
            } else {
                s.flowEff = 0.5 * Rules.CAPACITY[Rules.indexOfDn(s.dn)];
                assumedFlows++;
            }
        }
        if (assumedFlows > 0) {
            assumptions.add("У " + assumedFlows + " существующих участков нет flow_tph: принят расход, равный половине "
                    + "пропускной способности их диаметра. Проверка достаточности сети и реконструкция зависят от этого "
                    + "допущения; с реальными расходами результат будет точнее.");
        }

        // Камеры: число примыкающих участков и диаметр (если не задан — наибольший диаметр примыкающих участков).
        int assumedChamberDn = 0;
        for (Chamber c : chambers.values()) {
            int adjacent = 0;
            int maxDn = 0;
            if (c.upSeg != null && segments.containsKey(c.upstreamId)) {
                adjacent++;
                maxDn = Math.max(maxDn, c.upSeg.dn);
                c.neighbors.add(c.upSeg);
            }
            for (Segment s : segments.values()) {
                if (c.id.equals(s.upstreamId)) {
                    adjacent++;
                    maxDn = Math.max(maxDn, s.dn);
                    c.neighbors.add(s);
                }
            }
            c.adjacent = adjacent;
            if (c.dn != null) {
                c.dnEff = c.dn;
            } else {
                c.dnEff = maxDn > 0 ? maxDn : Rules.DN[0];
                assumedChamberDn++;
            }
        }
        if (assumedChamberDn > 0) {
            assumptions.add("У " + assumedChamberDn + " камер нет diameter: принят наибольший диаметр примыкающих участков.");
        }

        buildTargets();
    }

    /** Точка, к которой «привязан» участок сверху: камера, источник или ближайшая точка вышестоящего участка. */
    private double[] upstreamAnchor(Segment s) {
        String up = s.upstreamId;
        if (up == null) {
            return null;
        }
        if (chambers.containsKey(up)) {
            return chambers.get(up).xy;
        }
        if (sources.containsKey(up)) {
            return sources.get(up).xy;
        }
        Segment other = segments.get(up);
        if (other == null) {
            return null;
        }
        // Общий конец двух участков: берём тот конец текущего участка, который ближе к вышестоящему.
        double d0 = Geo.project(other.pts, s.pts[0][0], s.pts[0][1])[1];
        double d1 = Geo.project(other.pts, s.pts[s.pts.length - 1][0], s.pts[s.pts.length - 1][1])[1];
        return d0 <= d1 ? s.pts[0] : s.pts[s.pts.length - 1];
    }

    private Segment nextSegment(String id) {
        int guard = 0;
        while (id != null && guard++ < 1_000_000) {
            Segment s = segments.get(id);
            if (s != null) {
                return s;
            }
            Chamber c = chambers.get(id);
            if (c == null) {
                return null;
            }
            id = c.upstreamId;
        }
        return null;
    }

    private void buildTargets() {
        Map<String, Target> byKey = new LinkedHashMap<>();
        for (Map.Entry<String, Future> e : futures.entrySet()) {
            Target t = new Target();
            t.key = e.getKey();
            if (e.getValue().flow != null) {
                t.flow = e.getValue().flow;
            }
            byKey.put(t.key, t);
        }
        int withoutOksId = 0;
        for (RawPoint p : rawPoints) {
            String key = p.oksId != null ? p.oksId : p.id;
            if (p.oksId == null) {
                withoutOksId++;
            } else if (!futures.containsKey(key)) {
                diagnostics.add("У точки подключения " + p.id + " oks_id \"" + key + "\" не найден среди oks_future.");
            }
            Target t = byKey.computeIfAbsent(key, k -> {
                Target created = new Target();
                created.key = k;
                return created;
            });
            ConnectionPoint cp = new ConnectionPoint();
            cp.id = p.id;
            cp.xy = p.xy;
            t.points.add(cp);
            // Расход берётся из oks_future; если его нет — из самой точки подключения.
            if (t.flow == 0 && p.flow != null) {
                t.flow = p.flow;
            }
        }
        if (withoutOksId > 0) {
            assumptions.add("У " + withoutOksId + " точек подключения нет oks_id: каждая точка считается отдельным ОКС, "
                    + "его расход взят из flow_tph самой точки.");
        }
        for (Target t : byKey.values()) {
            if (t.points.isEmpty()) {
                t.problem = "у ОКС нет точки подключения (oks_connection_point)";
            } else if (t.flow <= 0) {
                t.problem = "не задан расчётный расход flow_tph";
            } else if (Rules.indexForFlow(t.flow) < 0) {
                t.problem = "расход " + t.flow + " т/ч больше пропускной способности самого большого диаметра";
            }
            targets.add(t);
        }
    }
}
