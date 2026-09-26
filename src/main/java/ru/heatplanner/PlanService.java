package ru.heatplanner;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.locationtech.jts.geom.Coordinate;
import org.springframework.stereotype.Service;
import ru.heatplanner.plan.Branch;
import ru.heatplanner.plan.Goals;
import ru.heatplanner.plan.JointPlanner;
import ru.heatplanner.plan.ObstacleSet;
import ru.heatplanner.plan.OutputBuilder;
import ru.heatplanner.plan.PlanInput;
import ru.heatplanner.plan.PlanValidator;
import ru.heatplanner.plan.RuleSet;
import ru.heatplanner.plan.TreeEvaluator;
import ru.heatplanner.plan.Variants;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.CompletableFuture;
import java.util.function.BiConsumer;

/**
 * Расчёт по загруженному файлу: основные варианты строго по данным и правилам и дополнительные с допущениями.
 * Подготовленные данные и графы видимости последнего файла хранятся в памяти, поэтому повторный расчёт того же файла
 * быстрее первого. Расчёты выполняются по одному (сервис заданий ставит их в очередь).
 */
@Service
public class PlanService {

    /** Результат расчёта: JSON для интерфейса и два выходных GeoJSON по разделу 7 (основные / основные + дополнительные). */
    public static final class PlanResult {
        public final String resultJson;
        public final String exportMain;
        public final String exportAll;

        PlanResult(String resultJson, String exportMain, String exportAll) {
            this.resultJson = resultJson;
            this.exportMain = exportMain;
            this.exportAll = exportAll;
        }
    }

    private static final class Prepared {
        final PlanInput input;
        final ObstacleSet obstacles;
        final JointPlanner planner;

        Prepared(PlanInput input, RuleSet rules) {
            this.input = input;
            this.obstacles = new ObstacleSet(input.obstacles, rules);
            this.planner = new JointPlanner(input, obstacles, rules);
        }
    }

    private final ObjectMapper mapper = new ObjectMapper();
    private String lastKey;
    private Prepared main;
    private Prepared extended;

    /** progress получает (процент, сообщение). */
    public synchronized PlanResult compute(Path file, Path roads, BiConsumer<Integer, String> progress) throws IOException {
        long started = System.currentTimeMillis();
        progress.accept(3, "Чтение и подготовка данных");
        String key = digest(file) + (roads == null ? "" : digest(roads));
        if (!key.equals(lastKey)) {
            lastKey = null;
            RuleSet strict = RuleSet.strict();
            RuleSet loose = strict.withOwnApproach(RuleSet.OwnApproachMode.NEAREST_VALID);
            PlanInput base = PlanInput.read(file, roads, strict, false);
            main = new Prepared(base, strict);
            extended = new Prepared(base.derive(loose, true), loose);
            lastKey = key;
        }
        progress.accept(10, "Расчёт вариантов");
        Prepared m = main;
        Prepared e = extended;
        CompletableFuture<List<Variants.Variant>> mainFuture = CompletableFuture.supplyAsync(
                () -> Variants.generate(m.input, m.obstacles, m.planner, 3, false));
        CompletableFuture<List<Variants.Variant>> extraFuture = CompletableFuture.supplyAsync(
                () -> Variants.generate(e.input, e.obstacles, e.planner, 3, true));
        List<Variants.Variant> mainVariants = mainFuture.join();
        progress.accept(80, "Основные варианты готовы, идёт расчёт дополнительных");
        List<Variants.Variant> additional = extraFuture.join();
        progress.accept(95, "Формирование результата");

        List<Object> described = new ArrayList<>();
        int rank = 1;
        for (Variants.Variant v : mainVariants) {
            described.add(describe(v, "v" + rank, rank++));
        }
        List<Object> extra = new ArrayList<>();
        rank = 1;
        for (Variants.Variant v : additional) {
            extra.add(describe(v, "a" + rank, rank++));
        }
        Map<String, Object> body = new LinkedHashMap<>();
        body.put("variants", described);
        body.put("additional", extra);
        List<String> diagnostics = new ArrayList<>(main.input.diagnostics);
        diagnostics.add(roads == null ? "Дороги OpenStreetMap не переданы: расчёт только по дорогам из файла."
                : "Дороги OpenStreetMap учтены в расчёте.");
        body.put("diagnostics", diagnostics);
        body.put("seconds", (System.currentTimeMillis() - started) / 1000.0);
        try {
            return new PlanResult(mapper.writeValueAsString(body),
                    mapper.writeValueAsString(OutputBuilder.build(mainVariants, new ArrayList<>())),
                    mapper.writeValueAsString(OutputBuilder.build(mainVariants, additional)));
        } catch (JsonProcessingException ex) {
            throw new IllegalStateException(ex);
        }
    }

    private static String digest(Path file) throws IOException {
        try {
            MessageDigest md = MessageDigest.getInstance("SHA-256");
            try (java.io.InputStream in = Files.newInputStream(file)) {
                byte[] buf = new byte[1 << 16];
                int n;
                while ((n = in.read(buf)) > 0) {
                    md.update(buf, 0, n);
                }
            }
            StringBuilder sb = new StringBuilder();
            for (byte b : md.digest()) {
                sb.append(String.format("%02x", b));
            }
            return sb.toString();
        } catch (NoSuchAlgorithmException ex) {
            throw new IllegalStateException(ex);
        }
    }
    private static double[] lonLat(Coordinate c) {
        double[] p = Utm.inverse(c.x, c.y);
        return new double[]{Math.round(p[0] * 1e7) / 1e7, Math.round(p[1] * 1e7) / 1e7};
    }

    private static Map<String, Object> feature(Map<String, Object> props, String type, Object coordinates) {
        Map<String, Object> geometry = new LinkedHashMap<>();
        geometry.put("type", type);
        geometry.put("coordinates", coordinates);
        Map<String, Object> f = new LinkedHashMap<>();
        f.put("type", "Feature");
        f.put("properties", props);
        f.put("geometry", geometry);
        return f;
    }

    private static Map<String, Object> collection(List<Object> features) {
        Map<String, Object> fc = new LinkedHashMap<>();
        fc.put("type", "FeatureCollection");
        fc.put("features", features);
        return fc;
    }

    /** Как подключена каждая цель: к сети или к ветви другой точки. */
    private static Map<String, String> attachments(JointPlanner.Solution s) {
        Map<String, String> out = new LinkedHashMap<>();
        for (Branch b : s.branches) {
            String what = b.parent >= 0 ? "в ветвь точки " + s.branches.get(b.parent).target.id.value()
                    : (b.existing.kind == Goals.Kind.NEW_CHAMBER ? "в существующую сеть (новая камера)" : "в существующую камеру");
            out.put(String.valueOf(b.target.id.value()), what);
        }
        for (JointPlanner.Unconnected u : s.unconnected) {
            out.put(String.valueOf(u.target.id.value()), "не подключена");
        }
        return out;
    }

    /** Что изменилось между прежней и новой структурой: цели, у которых изменилось подключение. */
    private static List<Object> changes(JointPlanner.Solution before, JointPlanner.Solution after) {
        Map<String, String> a = attachments(before);
        Map<String, String> b = attachments(after);
        List<Object> out = new ArrayList<>();
        for (Map.Entry<String, String> e : b.entrySet()) {
            String was = a.get(e.getKey());
            if (was != null && !was.equals(e.getValue())) {
                Map<String, Object> m = new LinkedHashMap<>();
                m.put("target_id", e.getKey());
                m.put("before", was);
                m.put("after", e.getValue());
                out.add(m);
            }
        }
        return out;
    }

    private static Map<String, Object> describe(Variants.Variant variant, String variantId, int rank) {
        return describe(variant, variantId, rank, true);
    }

    private static Map<String, Object> describe(Variants.Variant variant, String variantId, int rank, boolean withPrevious) {
        JointPlanner.Solution s = variant.solution;
        TreeEvaluator.Result ev = s.evaluation;
        List<Object> pieces = new ArrayList<>();
        Map<Integer, double[]> perBranch = new LinkedHashMap<>();
        for (TreeEvaluator.Piece p : ev.pieces) {
            Map<String, Object> props = new LinkedHashMap<>();
            props.put("target_id", s.branches.get(p.branch).target.id.value());
            props.put("flow_tph", Math.round(p.flow * 100) / 100.0);
            props.put("diameter", p.dn);
            props.put("length", Math.round(p.length * 10) / 10.0);
            props.put("cost", Math.round(p.cost));
            props.put("laying_method", s.branches.get(p.branch).route.legs.get(p.leg).special ? "special" : "base");
            pieces.add(feature(props, "LineString", Arrays.asList(lonLat(p.from), lonLat(p.to))));
            double[] agg = perBranch.computeIfAbsent(p.branch, k -> new double[]{0, 0, Double.MAX_VALUE, 0});
            agg[0] += p.length;
            agg[1] += p.cost;
            agg[2] = Math.min(agg[2], p.dn);
            agg[3] = Math.max(agg[3], p.dn);
        }
        List<Object> chambers = new ArrayList<>();
        for (TreeEvaluator.Chamber c : ev.chambers) {
            Map<String, Object> props = new LinkedHashMap<>();
            props.put("kind", c.existing ? "врезка в существующую камеру" : "новая камера");
            props.put("diameter", c.dn);
            props.put("cost", Math.round(c.cost));
            props.put("branch_target_id", s.branches.get(c.branch).target.id.value());
            chambers.add(feature(props, "Point", lonLat(c.xy)));
        }
        List<Object> branches = new ArrayList<>();
        for (int i = 0; i < s.branches.size(); i++) {
            Branch b = s.branches.get(i);
            double[] agg = perBranch.get(i);
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("target_id", b.target.id.value());
            m.put("flow_tph", b.target.flow);
            m.put("parent_target_id", b.parent < 0 ? null : s.branches.get(b.parent).target.id.value());
            m.put("connection", b.parent >= 0 ? "в ветвь точки " + s.branches.get(b.parent).target.id.value()
                    : (b.existing.kind == Goals.Kind.NEW_CHAMBER ? "в существующую сеть, новая камера"
                    : "в существующую камеру"));
            m.put("fallback_approach", !b.route.start.nearestBoundary);
            m.put("length", Math.round(agg[0] * 10) / 10.0);
            m.put("cost", Math.round(agg[1]));
            m.put("diameter_min", (int) agg[2]);
            m.put("diameter_max", (int) agg[3]);
            branches.add(m);
        }
        List<Object> unconnected = new ArrayList<>();
        for (JointPlanner.Unconnected u : s.unconnected) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("id", u.target.id.value());
            m.put("flow_tph", u.target.flow);
            m.put("reason", u.reason);
            m.put("coordinates", lonLat(u.target.xy));
            unconnected.add(m);
        }
        List<Object> violations = new ArrayList<>();
        int ruleCount = 0;
        for (PlanValidator.Violation x : variant.violations) {
            Map<String, Object> m = new LinkedHashMap<>();
            m.put("group", x.group.name());
            m.put("code", x.code);
            m.put("target_id", x.targetId);
            m.put("message", x.message);
            violations.add(m);
            ruleCount += x.group == PlanValidator.Group.RULE ? 1 : 0;
        }
        Map<String, Object> v = new LinkedHashMap<>();
        v.put("variant_id", variantId);
        v.put("rank", rank);
        v.put("strategy", variant.strategy);
        v.put("assumptions", variant.assumptions);
        v.put("routes", collection(pieces));
        v.put("chambers", collection(chambers));
        v.put("branches", branches);
        v.put("unconnected", unconnected);
        v.put("targets", s.branches.size() + s.unconnected.size());
        v.put("connected", s.branches.size());
        v.put("construction_cost", Math.round(ev.cost()));
        v.put("chamber_construction_cost", Math.round(ev.chamberCost));
        v.put("existing_chamber_tie_in_count", ev.existingTieIns);
        v.put("existing_chamber_tie_in_cost", Math.round(ev.tieInCost));
        v.put("unconnected_penalty", Math.round(s.penalty));
        v.put("calculated_cost", Math.round(s.calculatedCost()));
        v.put("new_network_length", Math.round(ev.length * 10) / 10.0);
        v.put("score", Math.round(s.score() * 10000) / 10000.0);
        v.put("rule_violations", ruleCount);
        v.put("structure_violations", violations.size() - ruleCount);
        v.put("violations", violations);
        if (withPrevious && variant.previous != null) {
            v.put("previous", describe(variant.previous, "p" + rank, rank, false));
            v.put("changes", changes(variant.previous.solution, variant.solution));
            @SuppressWarnings("unchecked")
            Map<String, Object> prev = (Map<String, Object>) v.get("previous");
            prev.put("changes", v.get("changes"));
        }
        return v;
    }
}