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
let resetMapState = () => {};
let currentResultTab = "input";
let syncRoadDemo = () => {};
let syncSolution = () => {};

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
function renderInputSummary(summary) {
    const counts = new Map();
    function add(key, count) {
        if (count > 0) counts.set(key, (counts.get(key) || 0) + count);
    }
    for (const [key, count] of Object.entries(summary.typeCounts || {})) {
        if (key !== "restriction") add(key, count);
    }
    let classifiedRestrictions = 0;
    for (const [key, count] of Object.entries(summary.restrictionCounts || {})) {
        add(key === "oks" ? "oks_existing" : `restriction:${key}`, count);
        classifiedRestrictions += count;
    }
    add("restriction", (summary.typeCounts?.restriction || 0) - classifiedRestrictions);
    const order = ["oks_existing", "source", "heat_network", "heat_chamber", "oks_connection_point"];
    const rank = key => order.includes(key) ? order.indexOf(key) : order.length;
    const entries = [...counts].sort(([a], [b]) => rank(a) - rank(b) || labelFor(a).localeCompare(labelFor(b), "ru"));
    countsInto("typeCounts", Object.fromEntries(entries), Object.fromEntries(entries.map(([key]) => [key, labelFor(key)])));
    renderDataWarnings(summary.warnings || []);
}
// Замечания сервера к данным: файл принят, но на что-то стоит обратить внимание (диагностика данных).
function renderDataWarnings(warnings) {
    document.getElementById("warningsHeading").hidden = !warnings.length;
    document.getElementById("dataWarnings").replaceChildren(...warnings.map(text => {
        const item = document.createElement("li");
        item.textContent = text;
        return item;
    }));
}

function setupResultTabs() {
    // Temporary UI placeholders. These are not calculated routes or exported results.
    const placeholderCount = 3;
    const tabs = [{id: "input", label: "Исходные данные", solutionNumber: null}];
    for (let number = 1; number <= placeholderCount; number++) {
        tabs.push({id: `solution-${number}`, label: placeholderCount === 1 ? "Решение" : `Решение ${number}`, solutionNumber: number});
    }
    // Дополнительные варианты (с допущениями) показываются только при включённом фильтре.
    for (let number = 1; number <= 3; number++) {
        tabs.push({id: `extra-${number}`, label: `Доп. вариант ${number}`, solutionNumber: placeholderCount + number, additional: true});
    }
    // Версии до перестройки цепочек: показываются фильтром «Прежние версии» для сравнения с итоговым решением.
    for (let number = 1; number <= 3; number++) {
        tabs.push({id: `prev-${number}`, label: `Было ${number}`, solutionNumber: 6 + number, previous: true});
    }
    const tablist = document.getElementById("resultTabs");
    const buttons = [];
    let selectedIndex = -1;
    function select(index, focus = false) {
        const tab = tabs[index];
        currentResultTab = tab.id;
        buttons.forEach((button, i) => {
            button.setAttribute("aria-selected", String(i === index));
            button.tabIndex = i === index ? 0 : -1;
        });
        document.getElementById("workspace").setAttribute("aria-labelledby", `tab-${tab.id}`);
        document.getElementById("inputSummary").hidden = tab.solutionNumber !== null;
        const summary = document.getElementById("solutionSummary");
        summary.hidden = tab.solutionNumber === null;
        document.getElementById("exportButton").hidden = tab.solutionNumber === null;
        document.getElementById("additionalToggleLabel").hidden = tab.solutionNumber === null;
        document.getElementById("previousToggleLabel").hidden = tab.solutionNumber === null;
        summary.textContent = tab.solutionNumber === null ? "" : `РЕШЕНИЕ НОМЕР ${tab.solutionNumber}`;
        document.querySelector(".statistics").scrollTop = 0;
        if (selectedIndex !== index) resetMapState();
        syncRoadDemo();
        syncSolution(tab.solutionNumber);
        selectedIndex = index;
        if (focus) buttons[index].focus();
    }
    tabs.forEach((tab, index) => {
        const button = document.createElement("button");
        button.type = "button";
        button.id = `tab-${tab.id}`;
        button.className = "result-tab";
        button.setAttribute("role", "tab");
        button.setAttribute("aria-controls", "workspace");
        button.textContent = tab.label;
        button.addEventListener("click", () => select(index));
        button.addEventListener("keydown", event => {
            const shown = buttons.map((b, i) => i).filter(i => !buttons[i].hidden);
            const at = shown.indexOf(index);
            let next;
            if (event.key === "ArrowRight") next = shown[(at + 1) % shown.length];
            else if (event.key === "ArrowLeft") next = shown[(at + shown.length - 1) % shown.length];
            else if (event.key === "Home") next = shown[0];
            else if (event.key === "End") next = shown[shown.length - 1];
            else return;
            event.preventDefault();
            select(next, true);
        });
        button.hidden = Boolean(tab.additional || tab.previous);
        buttons.push(button);
        tablist.append(button);
    });
    document.getElementById("additionalToggle").addEventListener("change", event => {
        buttons.forEach((button, i) => { if (tabs[i].additional) button.hidden = !event.target.checked; });
        if (!event.target.checked && tabs[selectedIndex].additional) select(0);
    });
    document.getElementById("previousToggle").addEventListener("change", event => {
        buttons.forEach((button, i) => { if (tabs[i].previous) button.hidden = !event.target.checked; });
        if (!event.target.checked && tabs[selectedIndex].previous) select(0);
    });
    select(0);
    tablist.hidden = false;
    document.getElementById("resultsToolbar").hidden = false;
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

function renderMap(geojson, workingDataset) {
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
    syncRoadDemo = mountRoadDemo(map, workingDataset, legendItems, () => currentResultTab === "input");
    syncSolution = mountPlan(map, window.uploadedFile, legendItems, document.getElementById("solutionSummary"));
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
    return () => {
        view.cancelAnimations();
        overlay.setPosition(undefined);
        map.getLayers().forEach(layer => layer.setVisible(true));
        legendItems.querySelectorAll('input[type="checkbox"]').forEach(check => { check.checked = true; });
        document.getElementById("legend").scrollTop = 0;
        fit();
    };
}

(async () => {
    // The old document shows the native leave warning. Only an accepted reload reaches here.
    if (performance.getEntriesByType("navigation")[0]?.type === "reload") {
        window.location.replace("/");
        return;
    }
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
        window.uploadedFile = file;
        document.getElementById("datasetName").textContent = file.name;
        document.getElementById("totalObjects").textContent = summary.totalObjects.toLocaleString("ru-RU");
        renderInputSummary(summary);
        setupResultTabs();
        document.getElementById("workspace").hidden = false;
        if (file.size > MAX_PREVIEW_BYTES) {
            document.getElementById("mapStatus").textContent = "Статистика загружена. Отображение файлов больше 100 МиБ пока не поддерживается.";
            document.getElementById("legend").hidden = true;
            return;
        }
        const geojson = JSON.parse(await file.text());
        // Fit only after the font and visible page layout have their final dimensions.
        await document.fonts.ready;
        await new Promise(resolve => requestAnimationFrame(resolve));
        window.workingDataset = roadEnrichment.create(geojson);
        resetMapState = renderMap(geojson, window.workingDataset);
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
