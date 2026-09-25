package ru.heatplanner;

import com.fasterxml.jackson.databind.JsonNode;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Разбирает существующую сеть (source, heat_network, heat_chamber) по актуальному техническому приложению
 * и проверяет её данные. Связей по идентификаторам в модели нет: сеть определяется только геометрией.
 *
 * Что проверяется:
 *  - ошибки (файл отклоняется): у участка нет диаметра или диаметр не из таблицы 1;
 *  - предупреждения (файл принимается, замечание показывается пользователю): нет источника или он не один,
 *    камера не лежит на сети, у камеры больше четырёх примыканий, участки не связаны с источником,
 *    в сети замкнутые контуры, диаметр уменьшается к источнику.
 *
 * Примыкания к камере считаются по правилу приложения (п. 2.1, разъяснение 12): каждый линейный участок,
 * геометрически заканчивающийся в камере, — одно примыкание; линия, проходящая через камеру, разделяется ею
 * на две части и занимает два примыкания. Все расстояния считаются в метрах (EPSG:32637).
 */
public class ExistingNetworkBuilder {

    /** Допуск совпадения точек (камера на линии, конец линии у камеры), м. */
    static final double SNAP_TOLERANCE_M = 0.5;

    private static final int MAX_REPORTED_ERRORS = 20;
    private static final int MAX_REPORTED_IDS = 5;
    private static final int MAX_CHAMBER_SEGMENTS = 4;
    /** Размер ячейки индекса, м: заметно больше допуска, поэтому запрос идёт в одну ячейку. */
    private static final double GRID_M = 8.0;

    /** Допустимые условные диаметры, мм — таблица 1 технического приложения. */
    private static final Set<Integer> VALID_DIAMETERS = new HashSet<>(Arrays.asList(
            50, 65, 80, 100, 125, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000, 1200, 1400));

    private static final String SOURCE = "source";
    private static final String HEAT_NETWORK = "heat_network";
    private static final String HEAT_CHAMBER = "heat_chamber";

    /** Существующий участок: id для сообщений, диаметр и ломаная в метрах. */
    private static final class Line {
        final String label;
        final int diameter;
        final double[][] xy;

        Line(String label, int diameter, double[][] xy) {
            this.label = label;
            this.diameter = diameter;
            this.xy = xy;
        }
    }

    /** Камера или источник: id для сообщений и точка в метрах. */
    private static final class Pt {
        final String label;
        final double[] xy;

        Pt(String label, double[] xy) {
            this.label = label;
            this.xy = xy;
        }
    }

    /** Часть участка между двумя узлами: участок целиком либо его часть между камерами, стоящими на нём. */
    private static final class Edge {
        final Line line;
        final double[] from;
        final double[] to;

        Edge(Line line, double[] from, double[] to) {
            this.line = line;
            this.from = from;
            this.to = to;
        }
    }

    /** Сводка по существующей сети — уходит в JSON-ответ. */
    public static class NetworkSummary {
        public int sources;
        public int segments;
        public int chambers;
        public List<String> warnings;
    }

    private final List<Line> lines = new ArrayList<>();
    private final List<Pt> chambers = new ArrayList<>();
    private final List<Pt> sources = new ArrayList<>();
    private final List<String> errors = new ArrayList<>();
    private int errorCount;

    /**
     * Принимает очередной Feature; объекты других типов игнорирует. Геометрия уже проверена на форму и диапазон
     * (GeoJsonGeometryValidator) и на соответствие типу объекта (GeometryChecker), здесь она только переводится в метры.
     */
    public void add(JsonNode feature, String objectType, String id, long featureIndex) {
        if (!SOURCE.equals(objectType) && !HEAT_NETWORK.equals(objectType) && !HEAT_CHAMBER.equals(objectType)) {
            return;
        }
        String where = objectType + " \"" + id + "\" (Feature #" + featureIndex + ")";
        JsonNode coordinates = feature.path("geometry").path("coordinates");
        if (HEAT_NETWORK.equals(objectType)) {
            JsonNode d = feature.path("properties").path("diameter");
            if (d.isMissingNode() || d.isNull()) {
                error(where + ": не задан diameter.");
                return;
            }
            if (!d.isIntegralNumber()) {
                error(where + ": diameter должен быть целым числом.");
                return;
            }
            if (!VALID_DIAMETERS.contains(d.asInt())) {
                error(where + ": условный диаметр " + d.asInt() + " мм отсутствует в таблице 1.");
                return;
            }
            double[][] xy = new double[coordinates.size()][];
            for (int i = 0; i < xy.length; i++) {
                xy[i] = project(coordinates.get(i));
            }
            lines.add(new Line(id, d.asInt(), xy));
        } else {
            Pt p = new Pt(id, project(coordinates));
            (SOURCE.equals(objectType) ? sources : chambers).add(p);
        }
    }

    private static double[] project(JsonNode c) {
        return Utm.forward(c.get(0).asDouble(), c.get(1).asDouble());
    }

    /** Возвращает сводку с предупреждениями; ошибки данных собираются в одно исключение. */
    public NetworkSummary build() throws GeoJsonInspector.GeoJsonValidationException {
        throwIfErrors();
        NetworkSummary summary = new NetworkSummary();
        summary.sources = sources.size();
        summary.segments = lines.size();
        summary.chambers = chambers.size();
        List<String> warnings = new ArrayList<>();
        summary.warnings = warnings;

        if (sources.isEmpty()) {
            warnings.add("В файле нет источника теплоснабжения (source): связность сети с источником проверить нельзя.");
        } else if (sources.size() > 1) {
            warnings.add("В файле " + sources.size() + " источников, в базовой модели используется один.");
        }
        if (lines.isEmpty()) {
            warnings.add("В файле нет существующих участков тепловой сети (heat_network): подключаться некуда.");
            return summary;
        }

        List<Edge> edges = splitByChambers(warnings);
        checkGraph(edges, warnings);
        return summary;
    }

    // ---- Разбиение участков камерами ------------------------------------------------------------------------------

    /**
     * Камера, стоящая на участке не у его конца, делит участок на две части. Получается список рёбер сети;
     * для камер, не лежащих ни на одной линии, сразу выдаётся предупреждение.
     */
    private List<Edge> splitByChambers(List<String> warnings) {
        // Индекс отрезков по сетке: (участок, отрезок) во всех ячейках, куда попадает отрезок с запасом на допуск.
        Map<Long, List<long[]>> grid = new HashMap<>();
        for (int li = 0; li < lines.size(); li++) {
            double[][] xy = lines.get(li).xy;
            for (int si = 1; si < xy.length; si++) {
                long x0 = cell(Math.min(xy[si - 1][0], xy[si][0]) - SNAP_TOLERANCE_M);
                long x1 = cell(Math.max(xy[si - 1][0], xy[si][0]) + SNAP_TOLERANCE_M);
                long y0 = cell(Math.min(xy[si - 1][1], xy[si][1]) - SNAP_TOLERANCE_M);
                long y1 = cell(Math.max(xy[si - 1][1], xy[si][1]) + SNAP_TOLERANCE_M);
                for (long cx = x0; cx <= x1; cx++) {
                    for (long cy = y0; cy <= y1; cy++) {
                        grid.computeIfAbsent(key(cx, cy), k -> new ArrayList<>()).add(new long[]{li, si});
                    }
                }
            }
        }

        // Для каждого участка — камеры, стоящие на нём внутри (расстояние вдоль линии от начала и точка камеры).
        Map<Integer, List<double[]>> inside = new HashMap<>();
        List<String> floating = new ArrayList<>();
        int floatingCount = 0;
        for (Pt chamber : chambers) {
            boolean onNetwork = false;
            Map<Integer, double[]> best = new HashMap<>();
            for (long[] hit : grid.getOrDefault(key(cell(chamber.xy[0]), cell(chamber.xy[1])), new ArrayList<>())) {
                Line line = lines.get((int) hit[0]);
                int si = (int) hit[1];
                double d = Geo.pointSegment(chamber.xy[0], chamber.xy[1], line.xy[si - 1], line.xy[si]);
                if (d <= SNAP_TOLERANCE_M) {
                    onNetwork = true;
                    double along = alongLine(line.xy, si, chamber.xy);
                    double[] prev = best.get((int) hit[0]);
                    if (prev == null || d < prev[1]) {
                        best.put((int) hit[0], new double[]{along, d});
                    }
                }
            }
            for (Map.Entry<Integer, double[]> e : best.entrySet()) {
                Line line = lines.get(e.getKey());
                double total = Geo.length(line.xy);
                double along = e.getValue()[0];
                // У самого конца камера не делит участок: она просто стоит на его конце.
                if (along > SNAP_TOLERANCE_M && along < total - SNAP_TOLERANCE_M) {
                    inside.computeIfAbsent(e.getKey(), k -> new ArrayList<>())
                            .add(new double[]{along, chamber.xy[0], chamber.xy[1]});
                }
            }
            if (!onNetwork) {
                floatingCount++;
                if (floating.size() < MAX_REPORTED_IDS) {
                    floating.add(chamber.label);
                }
            }
        }
        if (floatingCount > 0) {
            warnings.add("Камеры не лежат на существующей сети (дальше " + SNAP_TOLERANCE_M + " м от любого участка): "
                    + floatingCount + " шт., например " + String.join(", ", floating) + ".");
        }

        List<Edge> edges = new ArrayList<>();
        for (int li = 0; li < lines.size(); li++) {
            Line line = lines.get(li);
            double[] first = line.xy[0];
            double[] last = line.xy[line.xy.length - 1];
            List<double[]> cuts = inside.getOrDefault(li, new ArrayList<>());
            cuts.sort((a, b) -> Double.compare(a[0], b[0]));
            double[] from = first;
            for (double[] cut : cuts) {
                double[] at = {cut[1], cut[2]};
                edges.add(new Edge(line, from, at));
                from = at;
            }
            edges.add(new Edge(line, from, last));
        }
        return edges;
    }

    /** Расстояние вдоль линии от её начала до проекции точки на отрезок si. */
    private static double alongLine(double[][] xy, int si, double[] p) {
        double sum = 0;
        for (int i = 1; i < si; i++) {
            sum += Geo.dist(xy[i - 1], xy[i]);
        }
        double[] a = xy[si - 1];
        double[] b = xy[si];
        double len2 = Math.pow(b[0] - a[0], 2) + Math.pow(b[1] - a[1], 2);
        double t = len2 == 0 ? 0 : Math.max(0, Math.min(1, ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / len2));
        return sum + t * Math.sqrt(len2);
    }

    // ---- Граф: связность, примыкания, диаметры ------------------------------------------------------------------------

    private void checkGraph(List<Edge> edges, List<String> warnings) {
        // Узлы — точки, лежащие ближе допуска друг к другу: концы рёбер, камеры и источники (union-find по сетке).
        List<double[]> anchors = new ArrayList<>();
        for (Edge e : edges) {
            anchors.add(e.from);
            anchors.add(e.to);
        }
        int edgeAnchors = anchors.size();
        for (Pt c : chambers) {
            anchors.add(c.xy);
        }
        for (Pt s : sources) {
            anchors.add(s.xy);
        }
        int[] parent = cluster(anchors);

        // Примыкания: сколько концов рёбер попало в узел камеры. Каждая часть линии — отдельное примыкание.
        Map<Integer, Integer> endpointsAt = new HashMap<>();
        for (int i = 0; i < edgeAnchors; i++) {
            endpointsAt.merge(find(parent, i), 1, Integer::sum);
        }
        List<String> overloaded = new ArrayList<>();
        int overloadedCount = 0;
        for (int i = 0; i < chambers.size(); i++) {
            int adjacent = endpointsAt.getOrDefault(find(parent, edgeAnchors + i), 0);
            if (adjacent > MAX_CHAMBER_SEGMENTS) {
                overloadedCount++;
                if (overloaded.size() < MAX_REPORTED_IDS) {
                    overloaded.add(chambers.get(i).label + " (" + adjacent + ")");
                }
            }
        }
        if (overloadedCount > 0) {
            warnings.add("К существующим камерам примыкает больше " + MAX_CHAMBER_SEGMENTS + " участков: "
                    + overloadedCount + " шт., например " + String.join(", ", overloaded)
                    + ". К таким камерам новую сеть подключить нельзя.");
        }

        if (sources.isEmpty()) {
            return;
        }

        // Обход в ширину от источников по рёбрам: что достижимо, есть ли контуры, уменьшается ли диаметр к источнику.
        Map<Integer, List<Integer>> edgesAt = new HashMap<>();
        for (int i = 0; i < edges.size(); i++) {
            edgesAt.computeIfAbsent(find(parent, 2 * i), k -> new ArrayList<>()).add(i);
            edgesAt.computeIfAbsent(find(parent, 2 * i + 1), k -> new ArrayList<>()).add(i);
        }
        Set<Integer> visitedNodes = new HashSet<>();
        Set<Integer> visitedEdges = new HashSet<>();
        Map<Integer, Edge> reachedBy = new HashMap<>();
        Deque<Integer> queue = new ArrayDeque<>();
        for (int i = 0; i < sources.size(); i++) {
            int node = find(parent, edgeAnchors + chambers.size() + i);
            if (visitedNodes.add(node)) {
                queue.add(node);
            }
        }
        int loops = 0;
        int shrinkCount = 0;
        List<String> shrinkExamples = new ArrayList<>();
        while (!queue.isEmpty()) {
            int node = queue.poll();
            Edge parentEdge = reachedBy.get(node);
            for (int ei : edgesAt.getOrDefault(node, new ArrayList<>())) {
                if (!visitedEdges.add(ei)) {
                    continue;
                }
                Edge edge = edges.get(ei);
                // Диаметр существующей сети не должен уменьшаться по направлению к источнику (базовая модель, п. 2.2).
                if (parentEdge != null && parentEdge.line != edge.line && parentEdge.line.diameter < edge.line.diameter) {
                    shrinkCount++;
                    if (shrinkExamples.size() < MAX_REPORTED_IDS) {
                        shrinkExamples.add(edge.line.label + " (" + edge.line.diameter + " мм) → "
                                + parentEdge.line.label + " (" + parentEdge.line.diameter + " мм)");
                    }
                }
                int a = find(parent, 2 * ei);
                int far = a == node ? find(parent, 2 * ei + 1) : a;
                if (far == node) {
                    continue;
                }
                if (visitedNodes.add(far)) {
                    reachedBy.put(far, edge);
                    queue.add(far);
                } else {
                    loops++;
                }
            }
        }

        Set<String> lost = new HashSet<>();
        for (int i = 0; i < edges.size(); i++) {
            if (!visitedEdges.contains(i)) {
                lost.add(edges.get(i).line.label);
            }
        }
        if (!lost.isEmpty()) {
            warnings.add("Не связаны ни с одним источником: участков " + lost.size() + ", например "
                    + String.join(", ", new ArrayList<>(lost).subList(0, Math.min(MAX_REPORTED_IDS, lost.size()))) + ".");
        }
        if (loops > 0) {
            warnings.add("В существующей сети найдено замкнутых контуров: " + loops
                    + ". В базовой модели путь от каждого участка к источнику один.");
        }
        if (shrinkCount > 0) {
            warnings.add("Диаметр уменьшается по направлению к источнику на участках: " + shrinkCount
                    + " (в базовой модели он не должен уменьшаться), например: " + String.join("; ", shrinkExamples) + ".");
        }
    }

    /** Union-find по сетке: точки ближе допуска друг к другу попадают в один узел. Возвращает массив «родителей». */
    private static int[] cluster(List<double[]> points) {
        int m = points.size();
        int[] parent = new int[m];
        for (int i = 0; i < m; i++) {
            parent[i] = i;
        }
        Map<Long, List<Integer>> grid = new HashMap<>();
        for (int i = 0; i < m; i++) {
            long cx = (long) Math.floor(points.get(i)[0] / SNAP_TOLERANCE_M);
            long cy = (long) Math.floor(points.get(i)[1] / SNAP_TOLERANCE_M);
            for (long dx = -1; dx <= 1; dx++) {
                for (long dy = -1; dy <= 1; dy++) {
                    List<Integer> bucket = grid.get(key(cx + dx, cy + dy));
                    if (bucket == null) {
                        continue;
                    }
                    for (int j : bucket) {
                        if (Geo.dist(points.get(i), points.get(j)) <= SNAP_TOLERANCE_M) {
                            union(parent, i, j);
                        }
                    }
                }
            }
            grid.computeIfAbsent(key(cx, cy), k -> new ArrayList<>()).add(i);
        }
        return parent;
    }

    private static long cell(double meters) {
        return (long) Math.floor(meters / GRID_M);
    }

    /** Ключ ячейки сетки: два целых в одно число для HashMap. */
    private static long key(long x, long y) {
        return x * 4_000_037L + y;
    }

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
