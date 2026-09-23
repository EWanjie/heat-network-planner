package ru.heatplanner;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Новая сеть одного варианта как лес деревьев: корень — врезка в существующую сеть,
 * листья — точки подключения ОКС, промежуточные узлы — тепловые камеры-развилки.
 *
 * Дерево хранит только геометрию и связи. Расходы, диаметры, разбиение на участки и стоимость
 * считает Evaluator, поэтому любое изменение дерева (подключить ОКС, снять ОКС) сразу даёт новую оценку.
 */
final class Tree {

    enum Kind {
        /** Врезка в существующую камеру. */
        TIE_CHAMBER,
        /** Врезка в существующий участок: в точке врезки строится новая камера. */
        TIE_SEGMENT,
        /** Новая камера-развилка на новой трубе. */
        JUNCTION,
        /** Точка подключения ОКС. */
        TERMINAL
    }

    static final class Node {
        final int id;
        final Kind kind;
        final double[] xy;
        Edge in;
        final List<Edge> out = new ArrayList<>();
        PlanModel.Chamber chamber;
        PlanModel.Segment segment;
        /** Для TIE_SEGMENT: расстояние от вышестоящего конца участка до точки врезки. */
        double along;
        PlanModel.Target target;
        PlanModel.ConnectionPoint point;

        Node(int id, Kind kind, double[] xy) {
            this.id = id;
            this.kind = kind;
            this.xy = xy;
        }

        boolean isTie() {
            return kind == Kind.TIE_CHAMBER || kind == Kind.TIE_SEGMENT;
        }

        /** Сколько участков уже примыкает к узлу (с существующими у врезок). */
        int adjacent() {
            switch (kind) {
                case TIE_CHAMBER:
                    return chamber.adjacent + out.size();
                case TIE_SEGMENT:
                    return 2 + out.size();
                case JUNCTION:
                    return 1 + out.size();
                default:
                    return 1;
            }
        }

        boolean canTakeBranch() {
            return kind != Kind.TERMINAL && adjacent() < Rules.MAX_CHAMBER_SEGMENTS;
        }
    }

    static final class Edge {
        Node from;
        Node to;
        /** Ломаная в направлении от корня к листу. */
        double[][] pts;
        double length;
        /** Зоны специального прохода вдоль ломаной; считаются один раз при создании участка. */
        List<Zones.Zone> zones;
    }

    final List<Node> nodes = new ArrayList<>();
    final List<Edge> edges = new ArrayList<>();
    private int nextId = 1;

    Node addNode(Kind kind, double[] xy) {
        Node n = new Node(nextId++, kind, xy);
        nodes.add(n);
        return n;
    }

    Node addTieChamber(PlanModel.Chamber chamber) {
        Node n = addNode(Kind.TIE_CHAMBER, chamber.xy);
        n.chamber = chamber;
        return n;
    }

    Node addTieSegment(PlanModel.Segment segment, double along, double[] xy) {
        Node n = addNode(Kind.TIE_SEGMENT, xy);
        n.segment = segment;
        n.along = along;
        return n;
    }

    Node addTerminal(PlanModel.Target target, PlanModel.ConnectionPoint point) {
        Node n = addNode(Kind.TERMINAL, point.xy);
        n.target = target;
        n.point = point;
        return n;
    }

    /** Соединяет узлы новой трубой (ломаная от from к to). */
    Edge connect(Node from, Node to, double[][] pts, PlanModel model) {
        Edge e = new Edge();
        e.from = from;
        e.to = to;
        e.pts = pts;
        e.length = Geo.length(pts);
        e.zones = Zones.of(pts, model);
        from.out.add(e);
        to.in = e;
        edges.add(e);
        return e;
    }

    /** Делит трубу в точке на расстоянии d от её начала: появляется камера-развилка. */
    Node split(Edge e, double d, PlanModel model) {
        double[] p = Geo.pointAt(e.pts, d);
        Node junction = addNode(Kind.JUNCTION, p);
        Node to = e.to;
        double[][] first = Geo.sub(e.pts, 0, d);
        double[][] second = Geo.sub(e.pts, d, e.length);
        e.from.out.remove(e);
        edges.remove(e);
        connect(e.from, junction, first, model);
        connect(junction, to, second, model);
        return junction;
    }

    /** Ищет узел-врезку в ту же камеру (одна камера — одна врезка). */
    Node findTie(PlanModel.Chamber chamber) {
        for (Node n : nodes) {
            if (n.kind == Kind.TIE_CHAMBER && n.chamber == chamber) {
                return n;
            }
        }
        return null;
    }

    List<Node> terminals() {
        List<Node> list = new ArrayList<>();
        for (Node n : nodes) {
            if (n.kind == Kind.TERMINAL) {
                list.add(n);
            }
        }
        return list;
    }

    boolean contains(PlanModel.Target target) {
        for (Node n : nodes) {
            if (n.kind == Kind.TERMINAL && n.target == target) {
                return true;
            }
        }
        return false;
    }

    /** Убирает ОКС из сети и подчищает: лишние развилки схлопываются, пустые врезки исчезают. */
    void remove(PlanModel.Target target, PlanModel model) {
        Node leaf = null;
        for (Node n : nodes) {
            if (n.kind == Kind.TERMINAL && n.target == target) {
                leaf = n;
            }
        }
        if (leaf == null) {
            return;
        }
        Node cur = leaf;
        while (cur != null) {
            Edge in = cur.in;
            if (in == null) {
                nodes.remove(cur);
                break;
            }
            Node parent = in.from;
            parent.out.remove(in);
            edges.remove(in);
            nodes.remove(cur);
            cur = null;
            if (parent.out.isEmpty() && parent.kind != Kind.TERMINAL) {
                cur = parent;
            } else if (parent.kind == Kind.JUNCTION && parent.out.size() == 1) {
                // Развилка с единственной веткой — просто продолжение трубы: склеиваем обе трубы в одну.
                Edge before = parent.in;
                Edge after = parent.out.get(0);
                Node grand = before.from;
                grand.out.remove(before);
                edges.remove(before);
                edges.remove(after);
                nodes.remove(parent);
                connect(grand, after.to, Geo.join(before.pts, after.pts), model);
            }
        }
    }

    /** Глубокая копия: геометрия (неизменяемые массивы) общая, связи новые. */
    Tree copy() {
        Tree t = new Tree();
        t.nextId = nextId;
        Map<Node, Node> map = new HashMap<>();
        for (Node n : nodes) {
            Node c = new Node(n.id, n.kind, n.xy);
            c.chamber = n.chamber;
            c.segment = n.segment;
            c.along = n.along;
            c.target = n.target;
            c.point = n.point;
            map.put(n, c);
            t.nodes.add(c);
        }
        for (Edge e : edges) {
            Edge c = new Edge();
            c.from = map.get(e.from);
            c.to = map.get(e.to);
            c.pts = e.pts;
            c.length = e.length;
            c.zones = e.zones;
            c.from.out.add(c);
            c.to.in = c;
            t.edges.add(c);
        }
        return t;
    }
}
