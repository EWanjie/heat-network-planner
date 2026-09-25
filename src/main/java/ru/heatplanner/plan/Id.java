package ru.heatplanner.plan;

import com.fasterxml.jackson.databind.JsonNode;

import java.util.Objects;

/**
 * Идентификатор объекта с сохранением типа: строка "1" и число 1 — разные объекты (раздел 1 приложения).
 * Формат значения не интерпретируется, идентификатор используется только как ключ и как ссылка в результате.
 */
public final class Id {

    private final boolean numeric;
    private final String text;

    private Id(boolean numeric, String text) {
        this.numeric = numeric;
        this.text = text;
    }

    public static Id of(JsonNode node) {
        if (node.isNumber()) {
            return new Id(true, node.isIntegralNumber() ? Long.toString(node.asLong()) : node.asText());
        }
        return new Id(false, node.asText());
    }

    public static Id text(String value) {
        return new Id(false, value);
    }

    public static Id number(long value) {
        return new Id(true, Long.toString(value));
    }

    /** Числовой ли идентификатор: при записи результата тип сохраняется таким же, как во входных данных. */
    public boolean isNumeric() {
        return numeric;
    }

    public String value() {
        return text;
    }

    @Override
    public boolean equals(Object o) {
        return o instanceof Id && ((Id) o).numeric == numeric && ((Id) o).text.equals(text);
    }

    @Override
    public int hashCode() {
        return Objects.hash(numeric, text);
    }

    /** Для сообщений: строки в кавычках, числа без. */
    @Override
    public String toString() {
        return numeric ? text : "\"" + text + "\"";
    }
}
