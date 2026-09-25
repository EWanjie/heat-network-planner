package ru.heatplanner.plan;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonToken;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.LinearRing;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.operation.buffer.BufferOp;
import org.locationtech.jts.operation.buffer.BufferParameters;
import ru.heatplanner.Utm;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.regex.Pattern;

/**
 * Входные данные расчёта в метрах (EPSG:32637): существующая сеть, цели, ограничения.
 *
 * Файл уже проверен GeoJsonInspector. Здесь только перевод в геометрию JTS. Объекты без правила в таблице 2
 * (необязательные типы) не используются, о них остаётся запись в diagnostics.
 * Существующие участки сети одновременно становятся ограничениями (пересечение без врезки — специальный проход).
 */
public final class PlanInput {

    public static final class Source {
        public final Id id;
        public final Coordinate xy;

        Source(Id id, Coordinate xy) {
            this.id = id;
            this.xy = xy;
        }
    }

    public static final class Segment {
        public final Id id;
        public final int dn;
        public final LineString line;

        Segment(Id id, int dn, LineString line) {
            this.id = id;
            this.dn = dn;
            this.line = line;
        }
    }

    public static final class Chamber {
        public final Id id;
        public final Coordinate xy;

        Chamber(Id id, Coordinate xy) {
            this.id = id;
            this.xy = xy;
        }
    }

    /** Цель подключения: точка подключения ОКС с собственным расходом. */
    public static final class Target {
        public final Id id;
        public final Coordinate xy;
        public final double flow;

        Target(Id id, Coordinate xy, double flow) {
            this.id = id;
            this.xy = xy;
            this.flow = flow;
        }
    }

    private static final Pattern WIDTH = Pattern.compile("^\\s*\\d+(?:\\.\\d+)?\\s*(?:m)?\\s*$");

    public final List<Source> sources = new ArrayList<>();
    public final List<Segment> segments = new ArrayList<>();
    public final List<Chamber> chambers = new ArrayList<>();
    public final List<Target> targets = new ArrayList<>();
    public final List<Obstacle> obstacles = new ArrayList<>();
    /** Замечания к входу: неподдержанные типы, дороги без ширины, дубли. */
    public final List<String> diagnostics = new ArrayList<>();

    private final GeometryFactory factory = new GeometryFactory();
    private final RuleSet rules;
    private final Set<String> unsupported = new LinkedHashSet<>();
    private int roadsWithoutWidth;
    private int roadsFromCenterline;

    private PlanInput(RuleSet rules) {
        this.rules = rules;
    }

    public static PlanInput read(Path file, RuleSet rules) throws IOException {
        try (InputStream in = Files.newInputStream(file)) {
            return read(in, rules);
        }
    }

    public static PlanInput read(InputStream in, RuleSet rules) throws IOException {
        PlanInput input = new PlanInput(rules);
        ObjectMapper mapper = new ObjectMapper();
        try (JsonParser parser = mapper.getFactory().createParser(in)) {
            parser.nextToken();
            while (parser.nextToken() == JsonToken.FIELD_NAME) {
                String field = parser.currentName();
                parser.nextToken();
                if ("features".equals(field)) {
                    while (parser.nextToken() == JsonToken.START_OBJECT) {
                        input.add(mapper.readTree(parser));
                    }
                } else {
                    parser.skipChildren();
                }
            }
        }
        input.finish();
        return input;
    }

    private void add(JsonNode feature) {
        JsonNode props = feature.path("properties");
        JsonNode geometry = feature.path("geometry");
        Id id = Id.of(props.path("id"));
        switch (props.path("object_type").asText()) {
            case "source":
                sources.add(new Source(id, point(geometry)));
                break;
            case "heat_network": {
                int dn = props.path("diameter").asInt();
                if (Rules.indexOfDn(dn) < 0) {
                    diagnostics.add("Участок " + id + ": диаметр " + dn + " мм отсутствует в таблице 1, участок пропущен.");
                    break;
                }
                segments.add(new Segment(id, dn, line(geometry.path("coordinates"))));
                break;
            }
            case "heat_chamber":
                chambers.add(new Chamber(id, point(geometry)));
                break;
            case "oks_connection_point":
                targets.add(new Target(id, point(geometry), props.path("flow_tph").asDouble()));
                break;
            case "restriction":
                addRestriction(id, props, geometry);
                break;
            default:
                break;
        }
    }

    private void addRestriction(Id id, JsonNode props, JsonNode geometry) {
        String type = props.path("restriction_type").asText();
        Rules.RestrictionRule rule = Rules.RESTRICTIONS.get(type);
        if (rule == null) {
            unsupported.add(type);
            return;
        }
        String source = props.path("source").asText("input");
        Geometry geom = geometry(geometry);
        if (geom == null) {
            return;
        }
        boolean linear = geom.getDimension() == 1;
        if (rule.kind == Rules.Kind.SPECIAL_AREA && linear) {
            // Дорога или трамвай, заданные линией: без ширины границы полосы неизвестны.
            double width = width(props);
            if (width > 0) {
                BufferParameters p = new BufferParameters(rules.bufferQuadrantSegments, BufferParameters.CAP_FLAT,
                        BufferParameters.JOIN_ROUND, 5.0);
                obstacles.add(new Obstacle(obstacles.size(), id, rule, BufferOp.bufferOp(geom, width / 2, p), 0,
                        source, false, true));
                roadsFromCenterline++;
            } else {
                obstacles.add(new Obstacle(obstacles.size(), id, rule, geom, 0, source, true, false));
                roadsWithoutWidth++;
            }
            return;
        }
        obstacles.add(new Obstacle(obstacles.size(), id, rule, geom, 0, source, false, false));
    }

    /** Полная ширина дороги, м: атрибут width_m либо width вида «7.5» или «7.5 m»; 0, если ширины нет. */
    private static double width(JsonNode props) {
        JsonNode w = props.path("width_m");
        if (w.isNumber() && w.asDouble() > 0) {
            return w.asDouble();
        }
        JsonNode raw = props.path("width");
        if (raw.isNumber() && raw.asDouble() > 0) {
            return raw.asDouble();
        }
        if (raw.isTextual() && WIDTH.matcher(raw.asText()).matches()) {
            double v = Double.parseDouble(raw.asText().replaceAll("[^0-9.]", ""));
            return v > 0 ? v : 0;
        }
        return 0;
    }

    private void finish() {
        // Существующая сеть — препятствие с собственным габаритом: пересечение без врезки — специальный проход.
        Rules.RestrictionRule network = Rules.RESTRICTIONS.get("heat_network");
        for (Segment s : segments) {
            obstacles.add(new Obstacle(obstacles.size(), s.id, network, s.line, Rules.pairWidth(s.dn),
                    "existing_network", false, false));
        }
        if (!unsupported.isEmpty()) {
            diagnostics.add("Типы ограничений без правила в таблице 2 не используются в расчёте: "
                    + String.join(", ", unsupported) + ".");
        }
        if (roadsFromCenterline > 0) {
            diagnostics.add("Дорог, заданных осью и шириной: " + roadsFromCenterline
                    + ". Расчётная полоса — буфер на половину ширины, это модель, а не измеренная граница проезжей части.");
        }
        if (roadsWithoutWidth > 0) {
            diagnostics.add("ROAD_WIDTH_REQUIRED: дорог без ширины " + roadsWithoutWidth
                    + ". Ширина не выдумывается: такие дороги можно только обойти, специальный проход через них не рассчитывается.");
        }
    }

    // ---- GeoJSON -> JTS в метрах ------------------------------------------------------------------------------------

    private static Coordinate position(JsonNode c) {
        double[] xy = Utm.forward(c.get(0).asDouble(), c.get(1).asDouble());
        return new Coordinate(xy[0], xy[1]);
    }

    private Coordinate point(JsonNode geometry) {
        return position(geometry.path("coordinates"));
    }

    private LineString line(JsonNode coordinates) {
        Coordinate[] cs = new Coordinate[coordinates.size()];
        for (int i = 0; i < cs.length; i++) {
            cs[i] = position(coordinates.get(i));
        }
        return factory.createLineString(cs);
    }

    private Polygon polygon(JsonNode rings) {
        LinearRing shell = ring(rings.get(0));
        LinearRing[] holes = new LinearRing[rings.size() - 1];
        for (int i = 1; i < rings.size(); i++) {
            holes[i - 1] = ring(rings.get(i));
        }
        return factory.createPolygon(shell, holes);
    }

    private LinearRing ring(JsonNode coordinates) {
        Coordinate[] cs = new Coordinate[coordinates.size()];
        for (int i = 0; i < cs.length; i++) {
            cs[i] = position(coordinates.get(i));
        }
        return factory.createLinearRing(cs);
    }

    /** Геометрия ограничения: линия, мультилиния, полигон или мультиполигон; иначе null. */
    private Geometry geometry(JsonNode g) {
        JsonNode c = g.path("coordinates");
        switch (g.path("type").asText()) {
            case "LineString":
                return line(c);
            case "MultiLineString": {
                LineString[] lines = new LineString[c.size()];
                for (int i = 0; i < lines.length; i++) {
                    lines[i] = line(c.get(i));
                }
                return factory.createMultiLineString(lines);
            }
            case "Polygon":
                return polygon(c);
            case "MultiPolygon": {
                Polygon[] polygons = new Polygon[c.size()];
                for (int i = 0; i < polygons.length; i++) {
                    polygons[i] = polygon(c.get(i));
                }
                return factory.createMultiPolygon(polygons);
            }
            default:
                return null;
        }
    }
}
