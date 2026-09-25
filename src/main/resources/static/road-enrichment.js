/* Supplemental OSM restrictions, kept in memory. Never rewrites the uploaded file.
 * Road centerlines are NOT measured carriageway polygons. A future route calculator
 * must resolve missing widths before treating them as road areas.
 */
(function (root) {
    const HIGHWAYS = new Set(["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
        "secondary", "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential", "living_street", "service"]);
    function bounds(collection) {
        const box = [Infinity, Infinity, -Infinity, -Infinity];
        function coordinates(value) {
            if (!Array.isArray(value)) return;
            if (typeof value[0] === "number") {
                const [x, y] = value;
                if (!Number.isFinite(x) || !Number.isFinite(y) || Math.abs(x) > 180 || Math.abs(y) > 90) throw new Error("Некорректные координаты для загрузки дорог.");
                box[0] = Math.min(box[0], x); box[1] = Math.min(box[1], y);
                box[2] = Math.max(box[2], x); box[3] = Math.max(box[3], y);
            } else value.forEach(coordinates);
        }
        function geometry(g) {
            if (!g) return;
            if (g.type === "GeometryCollection") (g.geometries || []).forEach(geometry);
            else coordinates(g.coordinates);
        }
        collection.features.forEach(f => geometry(f.geometry));
        return box.every(Number.isFinite) ? box : null;
    }
    // Liang–Barsky: retain only the part of each centerline inside the dataset rectangle.
    function clipSegment(a, b, box) {
        let start = 0, end = 1;
        const dx = b[0] - a[0], dy = b[1] - a[1];
        const p = [-dx, dx, -dy, dy], q = [a[0] - box[0], box[2] - a[0], a[1] - box[1], box[3] - a[1]];
        for (let i = 0; i < 4; i++) {
            if (p[i] === 0) { if (q[i] < 0) return null; continue; }
            const r = q[i] / p[i];
            if (p[i] < 0) start = Math.max(start, r); else end = Math.min(end, r);
            if (start > end) return null;
        }
        if (start === end || (dx === 0 && dy === 0)) return null;
        return [[a[0] + start * dx, a[1] + start * dy], [a[0] + end * dx, a[1] + end * dy]];
    }
    const same = (a, b) => Math.abs(a[0] - b[0]) < 1e-10 && Math.abs(a[1] - b[1]) < 1e-10;
    function clipLine(coordinates, box) {
        const lines = []; let current = null;
        for (let i = 1; i < coordinates.length; i++) {
            const segment = clipSegment(coordinates[i - 1], coordinates[i], box);
            if (!segment) { current = null; continue; }
            if (current && same(current[current.length - 1], segment[0])) current.push(segment[1]);
            else { current = segment; lines.push(current); }
        }
        return lines;
    }
    function parseWidth(value) {
        if (typeof value !== "string" || !/^\s*\d+(?:\.\d+)?\s*(?:m)?\s*$/.test(value)) return null;
        const width = parseFloat(value);
        return width > 0 ? width : null;
    }
    function convert(response, bbox, original) {
        if (!Array.isArray(response.elements) || response.remark) throw new Error("Сервис OSM вернул неполный ответ. Повторите загрузку дорог.");
        const usedIds = new Set(original.features.map(f => String(f.properties?.id ?? f.id)));
        const existingOsm = new Set(original.features.filter(f => f.properties?.restriction_type === "road")
            .map(f => String(f.properties?.osm_id || "")));
        const seen = new Set(); const features = [];
        for (const way of response.elements) {
            if (way.type !== "way" || !HIGHWAYS.has(way.tags?.highway) || way.tags.area === "yes" || seen.has(way.id)) continue;
            seen.add(way.id);
            if (existingOsm.has(`way/${way.id}`) || existingOsm.has(String(way.id))) continue;
            if (!Array.isArray(way.geometry) || way.geometry.length < 2) throw new Error("В ответе OSM отсутствует геометрия дороги.");
            const coordinates = way.geometry.map(p => [p.lon, p.lat]);
            if (!coordinates.every(p => p.every(Number.isFinite))) throw new Error("В ответе OSM есть некорректная геометрия дороги.");
            const lines = clipLine(coordinates, bbox);
            if (!lines.length) continue;
            let id = `external:osm:way:${way.id}`;
            while (usedIds.has(id)) id += ":import";
            usedIds.add(id);
            const width = parseWidth(way.tags.width);
            features.push({type: "Feature", properties: {id, object_type: "restriction", restriction_type: "road",
                source: "OpenStreetMap", osm_id: `way/${way.id}`, name: way.tags.name || "", highway: way.tags.highway,
                geometry_role: "centerline", width_m: width, width_source: width === null ? "unknown" : "osm_width_tag",
                requires_road_area: true, bridge: way.tags.bridge || "no", tunnel: way.tags.tunnel || "no", layer: way.tags.layer || "0"},
                geometry: lines.length === 1 ? {type: "LineString", coordinates: lines[0]} : {type: "MultiLineString", coordinates: lines}});
        }
        return features;
    }
    function create(original) {
        const bbox = bounds(original);
        let state = "idle", roads = [], pending = null, error = "", fetchedAt = null;
        const listeners = new Set();
        const notify = () => listeners.forEach(fn => fn());
        function snapshot() { return {status: state, bbox: bbox?.slice() || null, count: roads.length, error, fetchedAt}; }
        async function load() {
            if (pending) return pending;
            if (state === "ready") return snapshot();
            pending = Promise.resolve().then(async () => {
                state = "loading"; error = ""; notify();
                const controller = new AbortController();
                const timeout = setTimeout(() => controller.abort(), 35000);
                try {
                    if (!bbox) { roads = []; state = "ready"; return snapshot(); }
                    const [west, south, east, north] = bbox;
                    if (west === east || south === north) throw new Error("Для загрузки дорог нужен прямоугольник ненулевой площади.");
                    const areaKm2 = (east - west) * (north - south) * 111.32 ** 2 * Math.cos((south + north) * Math.PI / 360);
                    if (east - west > 180 || areaKm2 > 100) throw new Error("Территория слишком велика для демонстрационной загрузки дорог (лимит 100 км²).");
                    const query = `[out:json][timeout:25];way["highway"~"^(${[...HIGHWAYS].join("|")})$"](${south},${west},${north},${east});out geom;`;
                    const response = await fetch("https://overpass-api.de/api/interpreter", {
                        method: "POST", body: new URLSearchParams({data: query}), signal: controller.signal});
                    if (!response.ok) throw new Error(`Сервис дорог OSM недоступен (HTTP ${response.status}).`);
                    const result = await response.json();
                    roads = convert(result, bbox, original); fetchedAt = new Date().toISOString(); state = "ready";
                } catch (e) {
                    state = "error";
                    error = e.name === "AbortError" ? "Сервис дорог OSM не ответил за 35 секунд." : e.message;
                } finally { clearTimeout(timeout); pending = null; notify(); }
                return snapshot();
            });
            return pending;
        }
        return {
            load, snapshot,
            subscribe(fn) { listeners.add(fn); return () => listeners.delete(fn); },
            getRoads() { return structuredClone({type: "FeatureCollection", features: roads}); },
            // Use this when the future solver accepts enriched input. Visibility never affects restrictions.
            getWorkingDataset() {
                if (state !== "ready") throw new Error("Загрузка дополнительных ограничений ещё не завершена.");
                return {...original, features: [...original.features, ...structuredClone(roads)]};
            }
        };
    }
    const api = {create, bounds, clipLine, convert, parseWidth};
    if (typeof module !== "undefined") module.exports = api;
    else root.roadEnrichment = api;
})(typeof window !== "undefined" ? window : globalThis);
