// A visual background only: OSM objects are not added to the uploaded dataset or calculations.
window.createSchematicBasemap = function (status) {
    const fill = color => new ol.style.Fill({color});
    const stroke = (color, width, lineDash) => new ol.style.Stroke({color, width, lineDash});
    const area = (color, outline, zIndex) => new ol.style.Style({fill: fill(color),
        stroke: outline ? stroke(outline, 1) : undefined, zIndex});
    const styles = {
        water: area("#303030", "#454545", 1),
        park: area("#242424", "#353535", 2),
        building: area("#363636", "#505050", 3),
        waterway: new ol.style.Style({stroke: stroke("#454545", 1.5), zIndex: 2}),
        rail: new ol.style.Style({stroke: stroke("#626262", 1.5, [7, 5]), zIndex: 5})
    };
    const roads = new Map();
    function roadStyle(kind, resolution) {
        const major = ["motorway", "trunk", "primary", "secondary"].includes(kind);
        const minor = ["path", "track", "service"].includes(kind);
        if (minor && resolution > 7) return;
        const width = major ? 5 : minor ? 1.5 : 3;
        const key = String(width);
        if (!roads.has(key)) roads.set(key, new ol.style.Style({stroke: stroke(major ? "#737373" : "#545454", width), zIndex: 4}));
        return roads.get(key);
    }
    function nameStyle(feature, line) {
        const name = feature.get("name:ru") || feature.get("name") || feature.get("name:latin");
        if (!name) return;
        return new ol.style.Style({zIndex: 10, text: new ol.style.Text({text: String(name),
            font: '12px "Montserrat", sans-serif', placement: line ? "line" : "point",
            overflow: false, repeat: line ? 350 : undefined,
            fill: fill("#b5b5b5"), stroke: stroke("#191919", 3), padding: [3, 3, 3, 3]})});
    }
    function style(feature, resolution) {
        const layer = feature.get("layer"), kind = feature.get("class");
        switch (layer) {
            case "water": return styles.water;
            case "waterway": return resolution < 15 ? styles.waterway : undefined;
            case "park": return resolution < 15 ? [styles.park, nameStyle(feature, false)].filter(Boolean) : styles.park;
            case "landcover": return ["wood", "grass"].includes(kind) ? styles.park : undefined;
            case "landuse": return kind === "park" ? styles.park : undefined;
            case "building": return resolution < 12 ? styles.building : undefined;
            case "transportation":
                if (kind === "rail") return styles.rail;
                if (["aerialway", "ferry"].includes(kind)) return;
                return roadStyle(kind, resolution);
            case "transportation_name":
                if (resolution > 12 || ["rail", "aerialway", "ferry", "path", "service", "track"].includes(kind)) return;
                return nameStyle(feature, true);
            default: return;
        }
    }
    const layer = new ol.layer.VectorTile({zIndex: 0, background: "#191919", style, declutter: true});
    let pending = false;
    let source;
    async function load() {
        if (pending || source || !layer.getVisible()) return;
        pending = true;
        status.textContent = "Загружаем схематичную карту…";
        const controller = new AbortController();
        const timeout = setTimeout(() => controller.abort(), 15000);
        try {
            const response = await fetch("https://tiles.openfreemap.org/planet", {signal: controller.signal});
            if (!response.ok) throw new Error("Map metadata unavailable");
            const metadata = await response.json();
            if (!metadata.tiles?.length) throw new Error("Missing vector tiles");
            source = new ol.source.VectorTile({format: new ol.format.MVT(), urls: metadata.tiles,
                minZoom: metadata.minzoom ?? 0, maxZoom: metadata.maxzoom ?? 14,
                attributions: '<a href="https://openfreemap.org/" target="_blank" rel="noopener">OpenFreeMap</a> · <a href="https://openmaptiles.org/" target="_blank" rel="noopener">© OpenMapTiles</a> · <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">© OpenStreetMap contributors</a>'});
            let failed = false;
            source.on("tileloaderror", () => {
                failed = true;
                if (layer.getVisible()) status.textContent = "Часть карты не загрузилась. Объекты файла доступны. Для повторной загрузки выключите и включите «Карта».";
            });
            source.on("tileloadend", () => {
                if (!failed) status.textContent = "";
            });
            layer.setSource(source);
            status.textContent = "";
        } catch (error) {
            if (layer.getVisible()) status.textContent = "Схематичная карта недоступна. Проверьте интернет и выключите/включите «Карта» для повтора. Объекты файла доступны.";
        } finally { clearTimeout(timeout); pending = false; }
    }
    layer.on("change:visible", () => {
        status.textContent = "";
        if (layer.getVisible()) {
            if (source) source.refresh();
            else load();
        }
    });
    load();
    return layer;
};
