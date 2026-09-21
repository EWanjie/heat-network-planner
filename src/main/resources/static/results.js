const OBJECT_LABELS = {
    source: "Источники теплоснабжения", heat_network: "Существующая теплосеть",
    heat_chamber: "Тепловые камеры", oks_connection_point: "Точки подключения ОКС",
    oks_existing: "Существующие ОКС", oks_future: "Перспективные ОКС", restriction: "Ограничения"
};
const RESTRICTION_LABELS = {
    oks: "Существующие ОКС", road: "Дороги", railway: "Железная дорога",
    tram_tracks: "Трамвайные пути", water: "Водные объекты", park: "Парки",
    social_area: "Социальные территории", prohibited_site: "Запрещённые территории",
    gas_pipeline: "Газопроводы", power_cable: "Силовые кабели"
};
const COLORS = {
    source: "#ff668c", heat_network: "#ff906c", heat_chamber: "#ffbb55",
    oks_connection_point: "#80f2a3", oks_existing: "#6c7887", oks_future: "#91a3b9",
    "restriction:oks": "#6c7887", "restriction:water": "#327bac",
    "restriction:railway": "#b38bdb", "restriction:road": "#a18e6c",
    "restriction:gas_pipeline": "#dbc878", "restriction:power_cable": "#bd81df",
    "restriction:park": "#477955", "restriction:social_area": "#bd818b",
    "restriction:prohibited_site": "#9b4b62"
};
const BUILDINGS = new Set(["oks_existing", "oks_future", "restriction:oks"]);
const TOP = new Set(["source", "heat_chamber", "oks_connection_point"]);
const MAX_PREVIEW_BYTES = 100 * 1024 * 1024;

function labelFor(key) {
    return key.startsWith("restriction:") ? (RESTRICTION_LABELS[key.slice(12)] || key.slice(12)) : (OBJECT_LABELS[key] || key);
}
function keyFor(feature) {
    const p = feature.properties || {};
    if (p.object_type === "oks_existing" || (p.object_type === "restriction" && p.restriction_type === "oks")) return "oks_existing";
    return p.object_type === "restriction" ? `restriction:${p.restriction_type}` : p.object_type;
}
function countsInto(id, counts, labels) {
    const list = document.getElementById(id);
    for (const [key, count] of Object.entries(counts)) {
        const item = document.createElement("li");
        const name = document.createElement("span");
        const value = document.createElement("strong");
        name.textContent = labels[key] || key;
        value.textContent = count.toLocaleString("ru-RU");
        item.append(name, value);
        list.append(item);
    }
    if (!list.children.length) list.textContent = "Нет объектов";
}
function popup(properties) {
    const table = document.createElement("table");
    table.className = "popup-table";
    for (const [key, value] of Object.entries(properties)) {
        const row = table.insertRow();
        row.insertCell().textContent = key;
        row.insertCell().textContent = typeof value === "object" ? JSON.stringify(value) : String(value);
    }
    return table;
}

function renderMap(geojson) {
    const view = new ol.View({center: [0, 0], zoom: 2, minZoom: 0, maxZoom: 19,
        enableRotation: false, smoothResolutionConstraint: false});
    const tileStatus = document.getElementById("tileStatus");
    const base = createSchematicBasemap(tileStatus);
    const map = new ol.Map({target: "map", layers: [base], view,
        interactions: ol.interaction.defaults.defaults({onFocusOnly: false}),
        controls: [new ol.control.Attribution({collapsible: false})]});
    const connectionPoints = new ol.source.Vector({features: new ol.format.GeoJSON().readFeatures({
        type: "FeatureCollection", features: geojson.features.filter(f => f.properties?.object_type === "oks_connection_point")
    }, {dataProjection: "EPSG:4326", featureProjection: "EPSG:3857"}), wrapX: false});
    function hasConnection(geometry) {
        if (!geometry || !["Polygon", "MultiPolygon"].includes(geometry.getType())) return false;
        return connectionPoints.getFeaturesInExtent(geometry.getExtent()).some(point => {
            const coordinate = point.getGeometry().getCoordinates();
            if (geometry.intersectsCoordinate(coordinate)) return true;
            // Include a connection exactly on the building boundary, without filling polygon holes.
            const nearest = geometry.getClosestPoint(coordinate);
            return Math.hypot(nearest[0] - coordinate[0], nearest[1] - coordinate[1]) < 0.000001;
        });
    }
    const groups = new Map();
    for (const feature of geojson.features) {
        const key = keyFor(feature);
        if (!groups.has(key)) groups.set(key, []);
        groups.get(key).push(feature);
    }
    const extent = ol.extent.createEmpty();
    const legendItems = document.getElementById("legendItems");
    function legendRow(key, title, color, layer) {
        const row = document.createElement("label");
        row.className = "legend-row";
        row.dataset.layer = key;
        const check = document.createElement("input");
        check.type = "checkbox";
        check.checked = true;
        check.setAttribute("aria-label", title);
        check.addEventListener("change", () => {
            layer.setVisible(check.checked);
            overlay.setPosition(undefined);
            if (key === "map") tileStatus.textContent = "";
        });
        const swatch = document.createElement("span");
        swatch.className = "swatch";
        swatch.style.backgroundColor = color;
        const name = document.createElement("span");
        name.textContent = title;
        row.append(check, swatch, name);
        legendItems.append(row);
    }
    const order = ["oks_existing", "source", "heat_network", "heat_chamber", "oks_connection_point"];
    const rank = key => order.includes(key) ? order.indexOf(key) : order.length;
    const sorted = [...groups].sort(([a], [b]) => rank(a) - rank(b) || labelFor(a).localeCompare(labelFor(b), "ru"));
    for (const [key, features] of sorted) {
        const color = COLORS[key] || "#c29976";
        const source = new ol.source.Vector({features: new ol.format.GeoJSON().readFeatures(
            {type: "FeatureCollection", features}, {dataProjection: "EPSG:4326", featureProjection: "EPSG:3857"}), wrapX: false});
        ol.extent.extend(extent, source.getExtent());
        const connected = new WeakSet();
        if (BUILDINGS.has(key)) source.getFeatures().forEach(feature => {
            if (hasConnection(feature.getGeometry())) connected.add(feature);
        });
        const cache = new Map();
        const layer = new ol.layer.Vector({source,
            zIndex: TOP.has(key) ? 500 : key === "heat_network" ? 400 : BUILDINGS.has(key) ? 200 : 300,
            style: feature => {
                const diameter = Number(feature.get("diameter"));
                const width = key === "heat_network" && Number.isFinite(diameter) ? Math.max(3, Math.min(7, 2 + diameter / 150)) : 1.5;
                const colorForFeature = connected.has(feature) ? COLORS.oks_connection_point : color;
                const styleKey = `${width}:${colorForFeature}`;
                if (!cache.has(styleKey)) cache.set(styleKey, new ol.style.Style({
                    stroke: new ol.style.Stroke({color: colorForFeature, width}),
                    fill: new ol.style.Fill({color: colorForFeature + (BUILDINGS.has(key) ? "4d" : "40")}),
                    image: new ol.style.Circle({radius: key === "source" ? 8 : 6,
                        fill: new ol.style.Fill({color}), stroke: new ol.style.Stroke({color: "#111820", width: 1.5})})
                }));
                return cache.get(styleKey);
            }});
        map.addLayer(layer);
        legendRow(key, labelFor(key), color, layer);
    }
    legendRow("map", "Карта", "#737373", base);
    const popupElement = document.createElement("div");
    popupElement.className = "map-popup";
    const close = document.createElement("button");
    close.type = "button"; close.textContent = "×"; close.setAttribute("aria-label", "Закрыть атрибуты");
    const content = document.createElement("div");
    popupElement.append(close, content);
    const overlay = new ol.Overlay({element: popupElement, positioning: "bottom-center", offset: [0, -12], autoPan: true});
    map.addOverlay(overlay);
    close.addEventListener("click", () => overlay.setPosition(undefined));
    map.on("singleclick", event => {
        const feature = map.forEachFeatureAtPixel(event.pixel, item => item, {hitTolerance: 5, layerFilter: layer => layer !== base});
        if (!feature) { overlay.setPosition(undefined); return; }
        const properties = {...feature.getProperties()};
        delete properties[feature.getGeometryName()];
        content.replaceChildren(popup(properties));
        overlay.setPosition(event.coordinate);
    });
    const controls = document.createElement("div");
    controls.className = "map-navigation ol-control";
    function control(id, title, icon, callback) {
        const button = document.createElement("button");
        button.type = "button"; button.id = id; button.title = title; button.setAttribute("aria-label", title);
        button.innerHTML = icon;
        button.addEventListener("click", callback);
        controls.append(button);
        return button;
    }
    const plus = control("zoomIn", "Приблизить", "+", () => view.setZoom(view.getZoom() + 1));
    const minus = control("zoomOut", "Отдалить", "−", () => view.setZoom(view.getZoom() - 1));
    const fitButton = control("fitMap", "Показать все объекты", '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.8" aria-hidden="true"><path d="M9 4H4v5m11-5h5v5M4 15v5h5m11-5v5h-5"/><rect x="8" y="8" width="8" height="8" rx="1"/></svg>', fit);
    map.addControl(new ol.control.Control({element: controls}));
    function fit() {
        map.updateSize();
        if (ol.extent.isEmpty(extent)) return;
        const small = map.getSize()[0] < 650;
        const legend = document.getElementById("legend");
        view.fit(extent, {padding: small ? [legend.offsetHeight + 32, 24, 42, 64] : [40, legend.offsetWidth + 40, 42, 64], maxZoom: 18});
    }
    function updateButtons() {
        plus.disabled = view.getZoom() >= view.getMaxZoom() - .001;
        minus.disabled = view.getZoom() <= view.getMinZoom() + .001;
    }
    fitButton.disabled = ol.extent.isEmpty(extent);
    if (!fitButton.disabled) {
        fit();
        // Allow three zoom levels beyond the full dataset, adapting to its geographic size.
        view.setMinZoom(Math.max(0, Math.floor(view.getZoom()) - 3));
    } else document.getElementById("mapStatus").textContent = "В файле нет объектов для отображения.";
    view.on("change:resolution", updateButtons);
    updateButtons();
}

(async () => {
    try {
        const data = await datasetStore.load();
        if (!data) { document.getElementById("emptyState").hidden = false; return; }
        // Native confirmation for reload, Back, close and navigation to upload.
        // Browsers choose the message and require prior user interaction with this page.
        window.addEventListener("beforeunload", event => {
            event.preventDefault();
            event.returnValue = "Данные могут не сохраниться";
        });
        const {file, summary} = data;
        document.getElementById("datasetName").textContent = file.name;
        document.getElementById("totalObjects").textContent = summary.totalObjects.toLocaleString("ru-RU");
        document.getElementById("fileSize").textContent = `${(summary.fileSizeBytes / 1024).toLocaleString("ru-RU", {maximumFractionDigits: 1})} КиБ`;
        countsInto("typeCounts", summary.typeCounts, OBJECT_LABELS);
        countsInto("restrictionCounts", summary.restrictionCounts, RESTRICTION_LABELS);
        document.getElementById("workspace").hidden = false;
        if (file.size > MAX_PREVIEW_BYTES) {
            document.getElementById("mapStatus").textContent = "Статистика загружена. Отображение файлов больше 100 МиБ пока не поддерживается.";
            document.getElementById("legend").hidden = true;
            return;
        }
        renderMap(JSON.parse(await file.text()));
    } catch (error) {
        if (document.getElementById("workspace").hidden) {
            document.getElementById("emptyState").hidden = false;
            document.getElementById("emptyMessage").textContent = error.message;
        } else {
            document.getElementById("mapStatus").textContent = "Не удалось отобразить карту: " + error.message;
            document.getElementById("mapStatus").classList.add("error");
        }
    }
})();
