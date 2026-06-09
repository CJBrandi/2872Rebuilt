package frc.robot;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

final class Json {
  private Json() {}

  static Map<String, Object> parseObject(String raw) {
    Object parsed = new Parser(raw).parse();
    if (!(parsed instanceof Map<?, ?> map)) {
      throw new IllegalArgumentException("Expected a JSON object");
    }
    Map<String, Object> result = new LinkedHashMap<>();
    for (var entry : map.entrySet()) {
      result.put(String.valueOf(entry.getKey()), entry.getValue());
    }
    return result;
  }

  static String stringify(Object value) {
    if (value == null) {
      return "null";
    }
    if (value instanceof String text) {
      return quote(text);
    }
    if (value instanceof Number || value instanceof Boolean) {
      return String.valueOf(value);
    }
    if (value instanceof Map<?, ?> map) {
      StringBuilder out = new StringBuilder("{");
      boolean first = true;
      for (var entry : map.entrySet()) {
        if (!first) {
          out.append(',');
        }
        first = false;
        out.append(quote(String.valueOf(entry.getKey())))
            .append(':')
            .append(stringify(entry.getValue()));
      }
      return out.append('}').toString();
    }
    if (value instanceof Iterable<?> values) {
      StringBuilder out = new StringBuilder("[");
      boolean first = true;
      for (Object item : values) {
        if (!first) {
          out.append(',');
        }
        first = false;
        out.append(stringify(item));
      }
      return out.append(']').toString();
    }
    return quote(String.valueOf(value));
  }

  private static String quote(String text) {
    StringBuilder out = new StringBuilder("\"");
    for (int i = 0; i < text.length(); i++) {
      char ch = text.charAt(i);
      switch (ch) {
        case '"' -> out.append("\\\"");
        case '\\' -> out.append("\\\\");
        case '\b' -> out.append("\\b");
        case '\f' -> out.append("\\f");
        case '\n' -> out.append("\\n");
        case '\r' -> out.append("\\r");
        case '\t' -> out.append("\\t");
        default -> {
          if (ch < 0x20) {
            out.append(String.format("\\u%04x", (int) ch));
          } else {
            out.append(ch);
          }
        }
      }
    }
    return out.append('"').toString();
  }

  private static final class Parser {
    private final String raw;
    private int index = 0;

    Parser(String raw) {
      this.raw = raw == null ? "" : raw;
    }

    Object parse() {
      Object value = parseValue();
      skipWhitespace();
      if (index != raw.length()) {
        throw error("Trailing JSON content");
      }
      return value;
    }

    private Object parseValue() {
      skipWhitespace();
      if (index >= raw.length()) {
        throw error("Unexpected end of JSON");
      }
      char ch = raw.charAt(index);
      if (ch == '{') {
        return parseMap();
      }
      if (ch == '[') {
        return parseList();
      }
      if (ch == '"') {
        return parseString();
      }
      if (raw.startsWith("true", index)) {
        index += 4;
        return true;
      }
      if (raw.startsWith("false", index)) {
        index += 5;
        return false;
      }
      if (raw.startsWith("null", index)) {
        index += 4;
        return null;
      }
      return parseNumber();
    }

    private Map<String, Object> parseMap() {
      Map<String, Object> result = new LinkedHashMap<>();
      expect('{');
      skipWhitespace();
      if (peek('}')) {
        index++;
        return result;
      }
      while (true) {
        String key = parseString();
        skipWhitespace();
        expect(':');
        result.put(key, parseValue());
        skipWhitespace();
        if (peek('}')) {
          index++;
          return result;
        }
        expect(',');
      }
    }

    private List<Object> parseList() {
      List<Object> result = new ArrayList<>();
      expect('[');
      skipWhitespace();
      if (peek(']')) {
        index++;
        return result;
      }
      while (true) {
        result.add(parseValue());
        skipWhitespace();
        if (peek(']')) {
          index++;
          return result;
        }
        expect(',');
      }
    }

    private String parseString() {
      expect('"');
      StringBuilder out = new StringBuilder();
      while (index < raw.length()) {
        char ch = raw.charAt(index++);
        if (ch == '"') {
          return out.toString();
        }
        if (ch != '\\') {
          out.append(ch);
          continue;
        }
        if (index >= raw.length()) {
          throw error("Unterminated escape");
        }
        char esc = raw.charAt(index++);
        switch (esc) {
          case '"' -> out.append('"');
          case '\\' -> out.append('\\');
          case '/' -> out.append('/');
          case 'b' -> out.append('\b');
          case 'f' -> out.append('\f');
          case 'n' -> out.append('\n');
          case 'r' -> out.append('\r');
          case 't' -> out.append('\t');
          case 'u' -> {
            if (index + 4 > raw.length()) {
              throw error("Short unicode escape");
            }
            out.append((char) Integer.parseInt(raw.substring(index, index + 4), 16));
            index += 4;
          }
          default -> throw error("Unsupported escape");
        }
      }
      throw error("Unterminated string");
    }

    private Number parseNumber() {
      int start = index;
      if (peek('-')) {
        index++;
      }
      while (index < raw.length() && Character.isDigit(raw.charAt(index))) {
        index++;
      }
      if (peek('.')) {
        index++;
        while (index < raw.length() && Character.isDigit(raw.charAt(index))) {
          index++;
        }
      }
      if (index < raw.length() && (raw.charAt(index) == 'e' || raw.charAt(index) == 'E')) {
        index++;
        if (index < raw.length() && (raw.charAt(index) == '+' || raw.charAt(index) == '-')) {
          index++;
        }
        while (index < raw.length() && Character.isDigit(raw.charAt(index))) {
          index++;
        }
      }
      String token = raw.substring(start, index);
      if (token.isEmpty() || "-".equals(token)) {
        throw error("Expected number");
      }
      return Double.valueOf(token);
    }

    private void skipWhitespace() {
      while (index < raw.length() && Character.isWhitespace(raw.charAt(index))) {
        index++;
      }
    }

    private boolean peek(char expected) {
      return index < raw.length() && raw.charAt(index) == expected;
    }

    private void expect(char expected) {
      skipWhitespace();
      if (!peek(expected)) {
        throw error("Expected '" + expected + "'");
      }
      index++;
    }

    private IllegalArgumentException error(String message) {
      return new IllegalArgumentException(message + " at byte " + index);
    }
  }
}
