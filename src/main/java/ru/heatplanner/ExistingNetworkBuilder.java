package ru.heatplanner;

import com.fasterxml.jackson.databind.JsonNode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * Собирает граф существующей сети из объектов source, heat_network и heat_chamber
 * и проверяет связи upstream_object_id: ссылка существует, нет циклов,
 * каждая цепочка приводит к источнику.
 */
public class ExistingNetworkBuilder {

    private static final int MAX_REPORTED_ERRORS = 20;
    private static final int MAX_CHAMBER_SEGMENTS = 4;

    /** Допустимые условные диаметры, мм — таблица 4.1 технического приложения. */
    private static final Set<Integer> VALID_DIAMETERS = new HashSet<>(Arrays.asList(
            50, 65, 80, 100, 125, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000, 1200, 1400));

    private static final String SOURCE = "source";
    private static final String HEAT_NETWORK = "heat_network";
    private static final String HEAT_CHAMBER = "heat_chamber";

    /** Узел графа. Геометрию пока не храним: она понадобится на этапе расчёта длин. */
    private static class Node {
        final String id;
        final String type;
        final Integer diameter;
        final Double flowTph;
        final String upstreamId;

        Node(String id, String type, Integer diameter, Double flowTph, String upstreamId) {
            this.id = id;
            this.type = type;
            this.diameter = diameter;
            this.flowTph = flowTph;
            this.upstreamId = upstreamId;
        }
    }

    /** Сводка по существующей сети — уходит в JSON-ответ. */
    public static class NetworkSummary {
        public int sources;
        public int segments;
        public int chambers;
        /** Самая длинная цепочка от источника, в объектах (источник — уровень 0). */
        public int maxDepth;
        /** Сколько объектов сети (включая сам источник) питается от каждого источника. */
        public Map<String, Integer> objectsPerSource;
        public List<String> warnings;
    }

    private final Map<String, Node> nodes = new LinkedHashMap<>();
    private final List<String> errors = new ArrayList<>();
    private int errorCount;

    /** Принимает очередной Feature; объекты других типов игнорирует. */
    public void add(JsonNode feature, String objectType, String id, long featureIndex) {
        if (!SOURCE.equals(objectType) && !HEAT_NETWORK.equals(objectType) && !HEAT_CHAMBER.equals(objectType)) {
            return;
        }
        String where = objectType + " \"" + id + "\" (Feature #" + featureIndex + ")";
        JsonNode properties = feature.path("properties");

        String geometryType = feature.path("geometry").path("type").asText();
        String expectedGeometry = HEAT_NETWORK.equals(objectType) ? "LineString" : "Point";
        if (!expectedGeometry.equals(geometryType)) {
            error(where + ": геометрия должна быть " + expectedGeometry + ", а не " + geometryType + ".");
        }

        Integer diameter = null;
        Double flow = null;
        String upstream = null;

        if (!SOURCE.equals(objectType)) {
            JsonNode d = properties.path("diameter");
            if (!d.isIntegralNumber()) {
                error(where + ": diameter должен быть целым числом.");
            } else if (!VALID_DIAMETERS.contains(d.asInt())) {
                error(where + ": условный диаметр " + d.asInt() + " мм отсутствует в таблице 4.1.");
            } else {
                diameter = d.asInt();
            }

            JsonNode u = properties.path("upstream_object_id");
            if ((u.isTextual() || u.isNumber()) && !u.asText().isBlank()) {
                upstream = u.asText();
            } else {
                error(where + ": не задан upstream_object_id.");
            }
        }

        if (HEAT_NETWORK.equals(objectType)) {
            JsonNode f = properties.path("flow_tph");
            if (!f.isNumber() || f.asDouble() < 0) {
                error(where + ": flow_tph должен быть неотрицательным числом.");
            } else {
                flow = f.asDouble();
            }
        }

        nodes.put(id, new Node(id, objectType, diameter, flow, upstream));
    }

    /** Проверяет связи и возвращает сводку. Все найденные ошибки собираются в одно исключение. */
    public NetworkSummary build() throws GeoJsonInspector.GeoJsonValidationException {
        // 1. Ссылки на upstream: объект существует и это не ссылка на самого себя.
        for (Node node : nodes.values()) {
            if (node.upstreamId == null) {
                continue;
            }
            if (node.upstreamId.equals(node.id)) {
                error(describe(node) + ": upstream_object_id указывает на сам объект.");
            } else if (!nodes.containsKey(node.upstreamId)) {
                error(describe(node) + ": upstream_object_id \"" + node.upstreamId
                        + "\" не найден среди source, heat_network и heat_chamber.");
            }
        }
        throwIfErrors();

        // 2. Каждая цепочка должна приводить к источнику без циклов.
        Map<String, String> rootOf = new HashMap<>();
        Map<String, Integer> depthOf = new HashMap<>();
        Set<String> broken = new HashSet<>();
        for (Node start : nodes.values()) {
            resolve(start, rootOf, depthOf, broken);
        }
        throwIfErrors();

        // 3. Сводка и предупреждения.
        NetworkSummary summary = new NetworkSummary();
        summary.objectsPerSource = new TreeMap<>();
        summary.warnings = new ArrayList<>();
        Map<String, Integer> adjacentSegments = new HashMap<>();

        for (Node node : nodes.values()) {
            if (SOURCE.equals(node.type)) {
                summary.sources++;
            } else if (HEAT_NETWORK.equals(node.type)) {
                summary.segments++;
            } else {
                summary.chambers++;
            }
            summary.objectsPerSource.merge(rootOf.get(node.id), 1, Integer::sum);
            summary.maxDepth = Math.max(summary.maxDepth, depthOf.get(node.id));

            // Участок примыкает к камере, если камера — его upstream или он — upstream камеры.
            Node up = node.upstreamId == null ? null : nodes.get(node.upstreamId);
            if (up != null && HEAT_NETWORK.equals(node.type) && HEAT_CHAMBER.equals(up.type)) {
                adjacentSegments.merge(up.id, 1, Integer::sum);
            }
            if (up != null && HEAT_CHAMBER.equals(node.type) && HEAT_NETWORK.equals(up.type)) {
                adjacentSegments.merge(node.id, 1, Integer::sum);
            }
        }
        adjacentSegments.forEach((chamberId, count) -> {
            if (count > MAX_CHAMBER_SEGMENTS) {
                summary.warnings.add("К камере \"" + chamberId + "\" примыкает " + count
                        + " участков, в ТЗ допускается не более " + MAX_CHAMBER_SEGMENTS + ".");
            }
        });
        return summary;
    }

    /** Идёт от узла вверх по upstream до источника (или уже обработанного узла) и проставляет источник и глубину. */
    private void resolve(Node start, Map<String, String> rootOf, Map<String, Integer> depthOf, Set<String> broken) {
        List<Node> path = new ArrayList<>();
        Set<String> onPath = new HashSet<>();
        Node cur = start;
        boolean cycle = false;

        while (!rootOf.containsKey(cur.id) && !broken.contains(cur.id)) {
            if (!onPath.add(cur.id)) {
                cycle = true;
                break;
            }
            path.add(cur);
            if (SOURCE.equals(cur.type)) {
                break;
            }
            cur = nodes.get(cur.upstreamId);
        }

        if (path.isEmpty()) {
            return;
        }
        if (cycle) {
            StringBuilder ids = new StringBuilder();
            boolean inCycle = false;
            for (Node n : path) {
                inCycle |= n.id.equals(cur.id);
                if (inCycle) {
                    ids.append(n.id).append(" -> ");
                }
            }
            error("Цикл в цепочке upstream_object_id: " + ids + cur.id + ".");
        }
        Node last = path.get(path.size() - 1);
        if (cycle || (!SOURCE.equals(last.type) && broken.contains(cur.id))) {
            path.forEach(n -> broken.add(n.id));
            return;
        }

        String rootId;
        int depth;
        if (SOURCE.equals(last.type)) {
            rootId = last.id;
            depth = -1;
        } else {
            rootId = rootOf.get(cur.id);
            depth = depthOf.get(cur.id);
        }
        for (int i = path.size() - 1; i >= 0; i--) {
            depth++;
            rootOf.put(path.get(i).id, rootId);
            depthOf.put(path.get(i).id, depth);
        }
    }

    private static String describe(Node node) {
        return node.type + " \"" + node.id + "\"";
    }

    private void error(String message) {
        errorCount++;
        if (errors.size() < MAX_REPORTED_ERRORS) {
            errors.add(message);
        }
    }

    private void throwIfErrors() throws GeoJsonInspector.GeoJsonValidationException {
        if (errorCount == 0) {
            return;
        }
        StringBuilder text = new StringBuilder("Ошибки в данных существующей сети (" + errorCount + "):");
        for (String e : errors) {
            text.append("\n- ").append(e);
        }
        if (errorCount > errors.size()) {
            text.append("\n... и ещё ").append(errorCount - errors.size());
        }
        throw new GeoJsonInspector.GeoJsonValidationException(text.toString());
    }
}
