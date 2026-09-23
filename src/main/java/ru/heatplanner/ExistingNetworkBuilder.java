package ru.heatplanner;

import com.fasterxml.jackson.databind.JsonNode;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

/**
 * Собирает граф существующей сети из объектов source, heat_network и heat_chamber.
 *
 * Два режима связей:
 * 1. Если в файле есть upstream_object_id, используются они (как в техническом приложении)
 *    и строго проверяются: ссылка существует, нет циклов, цепочка приводит к источнику.
 * 2. Если ни у одного объекта upstream_object_id нет, связи восстанавливаются по геометрии:
 *    концы линий, камеры и источники, лежащие ближе допуска друг к другу, считаются одним узлом,
 *    направление к источнику определяется обходом от источников.
 */
public class ExistingNetworkBuilder {

    /** Допуск совпадения точек при восстановлении связей по геометрии, м. */
    static final double SNAP_TOLERANCE_M = 0.5;

    private static final int MAX_REPORTED_ERRORS = 20;
    private static final int MAX_REPORTED_IDS = 5;
    private static final int MAX_CHAMBER_SEGMENTS = 4;

    /** Допустимые условные диаметры, мм — таблица 4.1 технического приложения. */
    private static final Set<Integer> VALID_DIAMETERS = new HashSet<>(Arrays.asList(
            50, 65, 80, 100, 125, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000, 1200, 1400));

    private static final String SOURCE = "source";
    private static final String HEAT_NETWORK = "heat_network";
    private static final String HEAT_CHAMBER = "heat_chamber";

    /**
     * Узел графа: объект входного файла (источник, участок или камера).
     * Необязательные атрибуты хранятся как null, если их нет в файле: что из этого ошибка, решает build().
     */
    private static class Node {
        final String id;
        final String type;
        /** Как назвать объект в сообщении об ошибке: тип, id и номер Feature в файле. */
        final String where;
        final Integer diameter;
        final Double flowTph;
        /** Следующий объект по направлению к источнику: из файла или восстановленный по геометрии. */
        String upstreamId;
        /** Для Point: одна пара координат; для LineString: первая и последняя. */
        final double[] start;
        final double[] end;

        Node(String id, String type, String where, Integer diameter, Double flowTph,
             String upstreamId, double[] start, double[] end) {
            this.id = id;
            this.type = type;
            this.where = where;
            this.diameter = diameter;
            this.flowTph = flowTph;
            this.upstreamId = upstreamId;
            this.start = start;
            this.end = end;
        }
    }

    /** Сводка по существующей сети — уходит в JSON-ответ. */
    public static class NetworkSummary {
        /** "attributes" — связи из upstream_object_id, "geometry" — восстановлены по геометрии. */
        public String linksMode;
        public int sources;
        public int segments;
        public int chambers;
        /** Самая длинная цепочка от источника, в объектах (источник — уровень 0). */
        public int maxDepth;
        /** Сколько объектов сети (включая сам источник) питается от каждого источника. */
        public Map<String, Integer> objectsPerSource;
        public List<String> warnings;
    }

    /** Все объекты сети по id, в порядке чтения файла. */
    private final Map<String, Node> nodes = new LinkedHashMap<>();
    /** Первые MAX_REPORTED_ERRORS ошибок для сообщения; errorCount считает все. */
    private final List<String> errors = new ArrayList<>();
    private final List<String> warnings = new ArrayList<>();
    private int errorCount;

    /**
     * Принимает очередной Feature; объекты других типов игнорирует.
     * Здесь проверяются только значения, которые в файле есть (тип геометрии, диаметр из таблицы 4.1,
     * неотрицательный расход). Обязательность атрибутов проверяется позже, в build(),
     * потому что она зависит от режима связей.
     */
    public void add(JsonNode feature, String objectType, String id, long featureIndex) {
        if (!SOURCE.equals(objectType) && !HEAT_NETWORK.equals(objectType) && !HEAT_CHAMBER.equals(objectType)) {
            return;
        }
        String where = objectType + " \"" + id + "\" (Feature #" + featureIndex + ")";
        JsonNode properties = feature.path("properties");
        JsonNode geometry = feature.path("geometry");

        String geometryType = geometry.path("type").asText();
        // Геометрия: участок — LineString (запоминаем первую и последнюю точки), камера и источник — Point.
        // Координаты нужны для восстановления связей по геометрии.
        boolean isLine = HEAT_NETWORK.equals(objectType);
        double[] start = null;
        double[] end = null;
        if (!(isLine ? "LineString" : "Point").equals(geometryType)) {
            error(where + ": геометрия должна быть " + (isLine ? "LineString" : "Point") + ", а не " + geometryType + ".");
        } else if (isLine) {
            JsonNode coords = geometry.path("coordinates");
            start = coordinate(coords.path(0));
            end = coordinate(coords.path(coords.size() - 1));
            if (coords.size() < 2 || start == null || end == null) {
                error(where + ": некорректные координаты LineString.");
                start = null;
            }
        } else {
            start = coordinate(geometry.path("coordinates"));
            end = start;
            if (start == null) {
                error(where + ": некорректные координаты Point.");
            }
        }

        // Атрибуты. Если поле есть, оно должно быть корректным; если поля нет, остаётся null.
        Integer diameter = null;
        Double flow = null;
        String upstream = null;

        JsonNode d = properties.path("diameter");
        if (!d.isMissingNode() && !d.isNull()) {
            if (!d.isIntegralNumber()) {
                error(where + ": diameter должен быть целым числом.");
            } else if (!VALID_DIAMETERS.contains(d.asInt())) {
                error(where + ": условный диаметр " + d.asInt() + " мм отсутствует в таблице 4.1.");
            } else {
                diameter = d.asInt();
            }
        }

        JsonNode f = properties.path("flow_tph");
        if (!f.isMissingNode() && !f.isNull()) {
            if (!f.isNumber() || f.asDouble() < 0) {
                error(where + ": flow_tph должен быть неотрицательным числом.");
            } else {
                flow = f.asDouble();
            }
        }

        JsonNode u = properties.path("upstream_object_id");
        if (!u.isMissingNode() && !u.isNull()) {
            if ((u.isTextual() || u.isNumber()) && !u.asText().isBlank()) {
                upstream = u.asText();
            } else {
                error(where + ": некорректный upstream_object_id.");
            }
        }

        nodes.put(id, new Node(id, objectType, where, diameter, flow, upstream, start, end));
    }

    /** Читает пару [долгота, широта]; null, если это не два числа. */
    private static double[] coordinate(JsonNode c) {
        if (c.isArray() && c.size() >= 2 && c.get(0).isNumber() && c.get(1).isNumber()) {
            return new double[]{c.get(0).asDouble(), c.get(1).asDouble()};
        }
        return null;
    }

    /**
     * Проверяет связи и возвращает сводку. Все найденные ошибки собираются в одно исключение.
     * Порядок: ошибки из add() -> выбор режима связей -> ссылки -> цепочки до источника -> сводка.
     */
    public NetworkSummary build() throws GeoJsonInspector.GeoJsonValidationException {
        throwIfErrors();

        // Выбор режима: есть хотя бы один upstream_object_id — файл по ТЗ, требуем атрибуты у всех;
        // ни одного — связи неизвестны, восстанавливаем их по геометрии.
        boolean anyUpstream = nodes.values().stream().anyMatch(n -> n.upstreamId != null);
        if (anyUpstream) {
            checkRequiredAttributes();
        } else {
            deriveLinksFromGeometry();
        }
        throwIfErrors();

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

        // 3. Сводка и предупреждения: считаем объекты, глубину и ветвление у камер.
        NetworkSummary summary = new NetworkSummary();
        summary.linksMode = anyUpstream ? "attributes" : "geometry";
        summary.objectsPerSource = new TreeMap<>();
        summary.warnings = new ArrayList<>(warnings);
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

        // По ТЗ (базовая модель) условный диаметр существующей сети не уменьшается по направлению к источнику.
        // Если в данных это не так, расчёт реконструкции будет опираться на противоречивые диаметры — предупреждаем.
        List<String> shrinking = new ArrayList<>();
        int shrinkingCount = 0;
        for (Node node : nodes.values()) {
            if (!HEAT_NETWORK.equals(node.type) || node.diameter == null) {
                continue;
            }
            Node up = nearestUpstreamSegment(node);
            if (up != null && up.diameter != null && up.diameter < node.diameter) {
                shrinkingCount++;
                if (shrinking.size() < MAX_REPORTED_IDS) {
                    shrinking.add(node.id + " (" + node.diameter + " мм) → " + up.id + " (" + up.diameter + " мм)");
                }
            }
        }
        if (shrinkingCount > 0) {
            summary.warnings.add("Диаметр уменьшается по направлению к источнику на участках: " + shrinkingCount
                    + " (в ТЗ он не должен уменьшаться), например: " + String.join("; ", shrinking) + ".");
        }
        return summary;
    }

    /**
     * Связи «объект → следующий по направлению к источнику» после build(): и заданные атрибутами,
     * и восстановленные по геометрии. Источники входят со значением null; исключённые из графа объекты отсутствуют.
     * Нужны расчётному ядру (Planner), чтобы не разбирать связи второй раз.
     */
    public Map<String, String> upstreamLinks() {
        Map<String, String> links = new LinkedHashMap<>();
        for (Node node : nodes.values()) {
            links.put(node.id, node.upstreamId);
        }
        return links;
    }

    /** Ближайший к источнику участок (heat_network) выше по цепочке; камеры пропускаются. Нет такого — null. */
    private Node nearestUpstreamSegment(Node node) {
        Node cur = node.upstreamId == null ? null : nodes.get(node.upstreamId);
        while (cur != null && !HEAT_NETWORK.equals(cur.type)) {
            if (SOURCE.equals(cur.type)) {
                return null;
            }
            cur = cur.upstreamId == null ? null : nodes.get(cur.upstreamId);
        }
        return cur;
    }

    /** Режим атрибутов: как в ТЗ, обязательны diameter, upstream_object_id и flow_tph участков. */
    private void checkRequiredAttributes() {
        for (Node node : nodes.values()) {
            if (SOURCE.equals(node.type)) {
                continue;
            }
            if (node.diameter == null) {
                error(node.where + ": не задан diameter.");
            }
            if (node.upstreamId == null) {
                error(node.where + ": не задан upstream_object_id.");
            }
            if (HEAT_NETWORK.equals(node.type) && node.flowTph == null) {
                error(node.where + ": не задан flow_tph.");
            }
        }
    }

    /**
     * Режим геометрии: узел — это группа точек (концы линий, камеры, источники) ближе допуска друг к другу.
     * Обход от источников по линиям проставляет upstream каждому объекту.
     */
    private void deriveLinksFromGeometry() {
        List<Node> sources = new ArrayList<>();
        for (Node n : nodes.values()) {
            if (SOURCE.equals(n.type)) {
                sources.add(n);
            }
        }
        if (sources.isEmpty()) {
            error("В файле нет объекта source, а upstream_object_id не заданы: направление к источнику определить нельзя.");
            return;
        }
        warnings.add("В файле нет upstream_object_id: связи восстановлены по геометрии (допуск совпадения точек "
                + SNAP_TOLERANCE_M + " м), направление к источнику определено обходом от источника.");

        // Шаг 1. Точки-якоря: у линии два (начало и конец), у камеры и источника один.
        // Координаты переводим из градусов в метры (равнопромежуточная проекция около широты источника):
        // для расстояний в десятки сантиметров точности хватает, полноценная проекция в EPSG:32637 будет позже.
        List<double[]> anchorXY = new ArrayList<>();
        List<Node> anchorOwner = new ArrayList<>();
        double lat0 = sources.get(0).start[1];
        double kx = 111320.0 * Math.cos(Math.toRadians(lat0));
        double ky = 110574.0;
        for (Node n : nodes.values()) {
            anchorXY.add(new double[]{n.start[0] * kx, n.start[1] * ky});
            anchorOwner.add(n);
            if (HEAT_NETWORK.equals(n.type)) {
                anchorXY.add(new double[]{n.end[0] * kx, n.end[1] * ky});
                anchorOwner.add(n);
            }
        }

        // Шаг 2. Кластеризация: якоря ближе допуска объединяются в один узел (union-find).
        // Сетка с шагом, равным допуску, ограничивает поиск соседей 9 ячейками, поэтому работает быстро и на больших файлах.
        int m = anchorXY.size();
        int[] parent = new int[m];
        for (int i = 0; i < m; i++) {
            parent[i] = i;
        }
        Map<Long, List<Integer>> grid = new HashMap<>();
        for (int i = 0; i < m; i++) {
            long cx = (long) Math.floor(anchorXY.get(i)[0] / SNAP_TOLERANCE_M);
            long cy = (long) Math.floor(anchorXY.get(i)[1] / SNAP_TOLERANCE_M);
            for (long dx = -1; dx <= 1; dx++) {
                for (long dy = -1; dy <= 1; dy++) {
                    List<Integer> cell = grid.get(cellKey(cx + dx, cy + dy));
                    if (cell == null) {
                        continue;
                    }
                    for (int j : cell) {
                        double ddx = anchorXY.get(i)[0] - anchorXY.get(j)[0];
                        double ddy = anchorXY.get(i)[1] - anchorXY.get(j)[1];
                        if (ddx * ddx + ddy * ddy <= SNAP_TOLERANCE_M * SNAP_TOLERANCE_M) {
                            union(parent, i, j);
                        }
                    }
                }
            }
            grid.computeIfAbsent(cellKey(cx, cy), k -> new ArrayList<>()).add(i);
        }

        // Шаг 3. Что лежит в каждом узле: pointsAt — камеры и источники, linesAt — концы линий,
        // lineEnds — для каждой линии номера узлов её начала и конца, pointCluster — узел камеры или источника.
        Map<Integer, List<Node>> pointsAt = new HashMap<>();
        Map<Integer, List<Node>> linesAt = new HashMap<>();
        Map<String, int[]> lineEnds = new HashMap<>();
        Map<String, Integer> pointCluster = new HashMap<>();
        for (int i = 0; i < m; i++) {
            Node n = anchorOwner.get(i);
            int cluster = find(parent, i);
            if (HEAT_NETWORK.equals(n.type)) {
                linesAt.computeIfAbsent(cluster, k -> new ArrayList<>()).add(n);
                int[] ends = lineEnds.computeIfAbsent(n.id, k -> new int[]{-1, -1});
                ends[ends[0] == -1 ? 0 : 1] = cluster;
            } else {
                pointsAt.computeIfAbsent(cluster, k -> new ArrayList<>()).add(n);
                pointCluster.put(n.id, cluster);
            }
        }

        // Шаг 4. Обход в ширину от источников. По ходу проставляем upstream:
        //  - линия, выходящая из узла, ссылается на камеру или источник в этом узле, а если их нет — на линию, по которой дошли до узла;
        //  - камера ссылается на линию, по которой до неё дошли.
        // Так направление «к источнику» получается из самой геометрии.
        Set<Integer> visitedClusters = new HashSet<>();
        Set<String> visitedLines = new HashSet<>();
        Map<Integer, Node> reachedBy = new HashMap<>();
        Deque<Integer> queue = new ArrayDeque<>();
        int loops = 0;
        for (Node s : sources) {
            int c = pointCluster.get(s.id);
            if (visitedClusters.add(c)) {
                queue.add(c);
            }
        }
        while (!queue.isEmpty()) {
            int c = queue.poll();
            List<Node> points = pointsAt.getOrDefault(c, new ArrayList<>());
            Node carrier = null;
            for (Node p : points) {
                if (SOURCE.equals(p.type)) {
                    carrier = p;
                    break;
                }
            }
            if (carrier == null && !points.isEmpty()) {
                carrier = points.get(0);
            }
            Node parentLine = reachedBy.get(c);
            if (carrier != null && !SOURCE.equals(carrier.type) && parentLine != null) {
                carrier.upstreamId = parentLine.id;
            }
            for (Node p : points) {
                if (p != carrier && !SOURCE.equals(p.type)) {
                    p.upstreamId = carrier != null ? carrier.id : null;
                }
            }
            String upstreamForLines = carrier != null ? carrier.id : (parentLine != null ? parentLine.id : null);
            for (Node line : linesAt.getOrDefault(c, new ArrayList<>())) {
                if (!visitedLines.add(line.id)) {
                    continue;
                }
                line.upstreamId = upstreamForLines;
                int[] ends = lineEnds.get(line.id);
                int far = ends[0] == c ? ends[1] : ends[0];
                if (far == c) {
                    continue;
                }
                if (visitedClusters.add(far)) {
                    reachedBy.put(far, line);
                    queue.add(far);
                } else {
                    // Дальний конец уже посещён другим путём — в сети замкнутый контур.
                    loops++;
                }
            }
        }

        // Шаг 5. Всё, до чего обход не дошёл, в граф не попадает: удаляем и предупреждаем.
        List<String> lost = new ArrayList<>();
        int lostLines = 0;
        int lostPoints = 0;
        for (Node n : new ArrayList<>(nodes.values())) {
            if (SOURCE.equals(n.type)) {
                continue;
            }
            boolean reached = HEAT_NETWORK.equals(n.type)
                    ? visitedLines.contains(n.id)
                    : visitedClusters.contains(pointCluster.get(n.id));
            if (!reached) {
                nodes.remove(n.id);
                if (lost.size() < MAX_REPORTED_IDS) {
                    lost.add(n.id);
                }
                if (HEAT_NETWORK.equals(n.type)) {
                    lostLines++;
                } else {
                    lostPoints++;
                }
            }
        }
        if (lostLines + lostPoints > 0) {
            warnings.add("Не связаны ни с одним источником и исключены из графа: участков " + lostLines
                    + ", камер " + lostPoints + " (например, " + String.join(", ", lost) + ").");
        }
        if (loops > 0) {
            warnings.add("В сети найдено замкнутых контуров: " + loops
                    + ". В ТЗ цепочка каждого участка должна вести к источнику, контур разорван условно.");
        }
        // Предупреждения об атрибутах, которых нет в файле, но которые понадобятся для расчёта.
        long noFlow = nodes.values().stream().filter(n -> HEAT_NETWORK.equals(n.type) && n.flowTph == null).count();
        if (noFlow > 0) {
            warnings.add("У участков существующей сети не задан flow_tph (" + noFlow
                    + " шт.): проверка пропускной способности и реконструкция без него невозможны.");
        }
        long noDiameter = nodes.values().stream().filter(n -> HEAT_CHAMBER.equals(n.type) && n.diameter == null).count();
        if (noDiameter > 0) {
            warnings.add("У камер не задан diameter (" + noDiameter + " шт.): для расчёта стоимости камер его нужно "
                    + "принять по наибольшему диаметру примыкающих участков.");
        }
    }

    /** Ключ ячейки сетки: два целых координат в одно число для HashMap. */
    private static long cellKey(long x, long y) {
        return x * 4_000_037L + y;
    }

    // Union-find: find возвращает представителя группы, union объединяет две группы.
    private static int find(int[] parent, int x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }

    private static void union(int[] parent, int a, int b) {
        int ra = find(parent, a);
        int rb = find(parent, b);
        if (ra != rb) {
            parent[ra] = rb;
        }
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
