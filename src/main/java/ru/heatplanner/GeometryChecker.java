package ru.heatplanner;

import com.fasterxml.jackson.databind.JsonNode;

/**
 * Проверка геометрии GeoJSON: форма координат, допустимый диапазон и соответствие типа объекта типу геометрии.
 *
 * Зачем: раньше сервер проверял только «geometry — это объект». Испорченная геометрия у ограничений или ОКС
 * проходила проверку, а карта потом падала непонятной ошибкой; координаты вне диапазона принимались молча.
 *
 * Методы возвращают описание первой найденной проблемы (без точки в конце) или null, если всё в порядке.
 * Исключения не бросаются: вызывающий код сам решает, копить ошибки или прерваться.
 */
final class GeometryChecker {

    private static final double MAX_LONGITUDE = 180.0;
    private static final double MAX_LATITUDE = 90.0;

    /** Допуск замкнутости кольца полигона, градусы (около 0,1 мм): убирает шум округления координат. */
    private static final double RING_CLOSURE_EPS = 1e-9;

    private GeometryChecker() {
    }

    /** Проверяет саму геометрию, не глядя на тип объекта. */
    static String check(JsonNode geometry) {
        String type = geometry.path("type").asText("");
        JsonNode coordinates = geometry.path("coordinates");
        switch (type) {
            case "Point":
                return position(coordinates);
            case "MultiPoint":
                return positions(coordinates, 1);
            case "LineString":
                return positions(coordinates, 2);
            case "MultiLineString":
                return each(coordinates, 1, "линий", line -> positions(line, 2));
            case "Polygon":
                return polygon(coordinates);
            case "MultiPolygon":
                return each(coordinates, 1, "полигонов", GeometryChecker::polygon);
            case "GeometryCollection":
                return each(geometry.path("geometries"), 1, "геометрий", GeometryChecker::check);
            case "":
                return "у геометрии не указан type";
            default:
                return "неизвестный тип геометрии «" + type + "»";
        }
    }

    /**
     * Проверяет, что тип геометрии подходит типу объекта (таблица 2.1 технического приложения).
     * Ограничения (restriction) могут иметь любую геометрию, поэтому для них проверки нет.
     */
    static String checkKind(String objectType, String geometryType) {
        switch (objectType) {
            case "source":
            case "heat_chamber":
            case "oks_connection_point":
                return geometryType.equals("Point") ? null : mismatch(objectType, "Point", geometryType);
            case "heat_network":
                return geometryType.equals("LineString") ? null : mismatch(objectType, "LineString", geometryType);
            case "oks_future":
            case "oks_existing":
                return geometryType.equals("Polygon") || geometryType.equals("MultiPolygon")
                        ? null : mismatch(objectType, "Polygon или MultiPolygon", geometryType);
            default:
                return null;
        }
    }

    private static String mismatch(String objectType, String expected, String actual) {
        return "объект типа " + objectType + " должен иметь геометрию " + expected + ", а не "
                + (actual.isEmpty() ? "пустую" : actual);
    }

    /** Одна позиция: массив из двух и более чисел [долгота, широта, (высота)] в допустимых пределах. */
    private static String position(JsonNode value) {
        if (!value.isArray() || value.size() < 2) {
            return "координата должна быть массивом из двух чисел [долгота, широта]";
        }
        for (JsonNode number : value) {
            if (!number.isNumber() || !Double.isFinite(number.asDouble())) {
                return "координаты должны быть конечными числами";
            }
        }
        double lon = value.get(0).asDouble();
        double lat = value.get(1).asDouble();
        if (Math.abs(lon) > MAX_LONGITUDE) {
            return "долгота " + lon + " вне диапазона от -180 до 180 (возможно, перепутаны долгота и широта)";
        }
        if (Math.abs(lat) > MAX_LATITUDE) {
            return "широта " + lat + " вне диапазона от -90 до 90 (возможно, перепутаны долгота и широта)";
        }
        return null;
    }

    /** Массив позиций: не меньше minimum точек, каждая корректна. */
    private static String positions(JsonNode values, int minimum) {
        if (!values.isArray()) {
            return "ожидается массив координат";
        }
        if (values.size() < minimum) {
            return "слишком мало точек: нужно не меньше " + minimum + ", а задано " + values.size();
        }
        for (JsonNode value : values) {
            String problem = position(value);
            if (problem != null) {
                return problem;
            }
        }
        return null;
    }

    /** Полигон: массив колец, каждое кольцо — не меньше 4 точек и замкнуто. */
    private static String polygon(JsonNode rings) {
        return each(rings, 1, "колец полигона", ring -> {
            String problem = positions(ring, 4);
            if (problem != null) {
                return problem;
            }
            JsonNode first = ring.get(0);
            JsonNode last = ring.get(ring.size() - 1);
            boolean closed = Math.abs(first.get(0).asDouble() - last.get(0).asDouble()) <= RING_CLOSURE_EPS
                    && Math.abs(first.get(1).asDouble() - last.get(1).asDouble()) <= RING_CLOSURE_EPS;
            return closed ? null : "кольцо полигона не замкнуто: первая и последняя точки не совпадают";
        });
    }

    /**
     * Применяет проверку к каждому элементу массива; сам массив должен содержать не меньше minimum элементов.
     * what — что лежит в массиве («колец полигона», «линий»…), для понятного текста ошибки.
     */
    private static String each(JsonNode array, int minimum, String what,
                               java.util.function.Function<JsonNode, String> rule) {
        if (!array.isArray() || array.size() < minimum) {
            return "ожидается непустой массив " + what;
        }
        for (JsonNode item : array) {
            String problem = rule.apply(item);
            if (problem != null) {
                return problem;
            }
        }
        return null;
    }
}
