package ru.heatplanner;

import com.fasterxml.jackson.databind.JsonNode;

/** Проверяет геометрию для отображения; не строит граф и не выполняет расчёты. */
public final class GeoJsonGeometryValidator {
    private GeoJsonGeometryValidator() {}
    public static void validate(JsonNode g, String where) throws GeoJsonInspector.GeoJsonValidationException {
        JsonNode c = g.path("coordinates");
        switch (g.path("type").asText()) {
            case "Point": position(c, where); break;
            case "MultiPoint": positions(c, 1, where); break;
            case "LineString": positions(c, 2, where); break;
            case "MultiLineString":
                array(c, 1, where);
                for (JsonNode line : c) positions(line, 2, where);
                break;
            case "Polygon": polygon(c, where); break;
            case "MultiPolygon":
                array(c, 1, where);
                for (JsonNode p : c) polygon(p, where);
                break;
            case "GeometryCollection":
                JsonNode children = g.path("geometries");
                array(children, 1, where);
                for (JsonNode child : children) validate(child, where);
                break;
            default: fail(where, "неподдерживаемый тип геометрии");
        }
    }
    private static void polygon(JsonNode rings, String where) throws GeoJsonInspector.GeoJsonValidationException {
        array(rings, 1, where);
        for (JsonNode ring : rings) {
            positions(ring, 4, where);
            JsonNode first = ring.get(0), last = ring.get(ring.size() - 1);
            if (first.size() != last.size()) fail(where, "кольцо полигона не замкнуто");
            for (int i = 0; i < first.size(); i++) {
                if (Double.compare(first.get(i).asDouble(), last.get(i).asDouble()) != 0)
                    fail(where, "кольцо полигона не замкнуто");
            }
        }
    }
    private static void positions(JsonNode values, int minimum, String where) throws GeoJsonInspector.GeoJsonValidationException {
        array(values, minimum, where);
        for (JsonNode value : values) position(value, where);
    }
    private static void position(JsonNode value, String where) throws GeoJsonInspector.GeoJsonValidationException {
        array(value, 2, where);
        for (JsonNode coordinate : value) {
            if (!coordinate.isNumber() || !Double.isFinite(coordinate.asDouble()))
                fail(where, "координаты должны быть конечными числами");
        }
        if (Math.abs(value.get(0).asDouble()) > 180 || Math.abs(value.get(1).asDouble()) > 90)
            fail(where, "координаты выходят за границы долготы и широты");
    }
    private static void array(JsonNode value, int minimum, String where) throws GeoJsonInspector.GeoJsonValidationException {
        if (!value.isArray() || value.size() < minimum) fail(where, "некорректный массив координат или геометрий");
    }
    private static void fail(String where, String message) throws GeoJsonInspector.GeoJsonValidationException {
        throw new GeoJsonInspector.GeoJsonValidationException(where + ": " + message + ".");
    }
}
