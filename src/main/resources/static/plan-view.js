// Результат расчёта на вкладках решений: слои трасс и камер, сводка, проверка и список точек поверх общей карты.
window.mountPlan = function mountPlan(map, file, legendItems, summaryElement) {
    const PALETTE = ["#ff668c", "#7fd1ff", "#ffd166", "#80f2a3", "#c792ea", "#ff9e64", "#4dd0c8", "#f78fb3", "#a3e635", "#f472b6"];
    const format = new ol.format.GeoJSON();
    const view = map.getView();
    const shortMoney = value => (value / 1e6).toLocaleString("ru-RU", {maximumFractionDigits: 1}) + " млн ₽";
    const diameterRange = b => b.diameter_min === b.diameter_max ? `ДУ ${b.diameter_min}` : `ДУ ${b.diameter_min}–${b.diameter_max}`;
    let state = "idle";
    let variants = [];
    let error = "";
    let currentSolution = null;
    let selectedTarget = null;
    const built = new Map();

    function element(tag, className, text) {
        const node = document.createElement(tag);
        if (className) node.className = className;
        if (text !== undefined) node.textContent = text;
        return node;
    }

    function addLegend(key, title, color, layer, group) {
        const row = element("label", "legend-row");
        row.dataset.layer = key;
        row.hidden = true;
        const check = document.createElement("input");
        check.type = "checkbox";
        check.checked = true;
        check.setAttribute("aria-label", title);
        check.addEventListener("change", () => layer.setVisible(check.checked));
        const swatch = element("span", "swatch");
        swatch.style.backgroundColor = color;
        row.append(check, swatch, element("span", "", title));
        legendItems.append(row);
        group.push({layer, row});
    }

    function build(index) {
        const variant = variants[index];
        const group = [];
        const features = format.readFeatures(variant.routes, {dataProjection: "EPSG:4326", featureProjection: "EPSG:3857"});
        const ids = [];
        variant.branches.forEach(b => ids.push(String(b.target_id)));
        const colorOf = id => PALETTE[Math.max(0, ids.indexOf(String(id))) % PALETTE.length];
        const casing = width => new ol.style.Style({stroke: new ol.style.Stroke({color: "#0d1117", width: width + 3.5, lineCap: "round"})});
        const cache = new Map();
        const routeLayer = new ol.layer.Vector({
            source: new ol.source.Vector({features, wrapX: false}), zIndex: 450, visible: false,
            style: feature => {
                const id = String(feature.get("target_id"));
                const width = Math.min(9, 3 + feature.get("diameter") / 60);
                const key = `${id}:${width}:${id === selectedTarget}`;
                if (!cache.has(key)) {
                    const color = colorOf(id);
                    const styles = [casing(width), new ol.style.Style({stroke: new ol.style.Stroke({
                        color, width, lineCap: "round", lineDash: feature.get("laying_method") === "special" ? [10, 8] : undefined})})];
                    if (id === selectedTarget) styles.unshift(new ol.style.Style({stroke: new ol.style.Stroke({color: "#ffffff", width: width + 8, lineCap: "round"})}));
                    cache.set(key, styles);
                }
                return cache.get(key);
            }});
        map.addLayer(routeLayer);
        addLegend(`plan-routes-${index}`, "Новые трассы (толщина — ДУ)", "#ff668c", routeLayer, group);
        const chamberLayer = new ol.layer.Vector({
            source: new ol.source.Vector({wrapX: false, features: format.readFeatures(variant.chambers,
                {dataProjection: "EPSG:4326", featureProjection: "EPSG:3857"})}), zIndex: 550, visible: false,
            style: new ol.style.Style({image: new ol.style.RegularShape({points: 4, radius: 8, angle: Math.PI / 4,
                fill: new ol.style.Fill({color: "#ffffff"}), stroke: new ol.style.Stroke({color: "#111820", width: 2})})})});
        map.addLayer(chamberLayer);
        addLegend(`plan-chambers-${index}`, "Новые камеры и врезки", "#ffffff", chamberLayer, group);
        const points = variant.unconnected.map(item => new ol.Feature({
            geometry: new ol.geom.Point(ol.proj.fromLonLat(item.coordinates)),
            target_id: item.id, flow_tph: item.flow_tph, reason: item.reason}));
        const badLayer = new ol.layer.Vector({
            source: new ol.source.Vector({features: points, wrapX: false}), zIndex: 600, visible: false,
            style: feature => new ol.style.Style({
                image: new ol.style.Circle({radius: 12, fill: new ol.style.Fill({color: "rgba(255,59,59,0.35)"}),
                    stroke: new ol.style.Stroke({color: "#ff3b3b", width: 3})}),
                text: new ol.style.Text({text: String(feature.get("target_id")), offsetY: -22, font: "600 14px sans-serif",
                    fill: new ol.style.Fill({color: "#ffd0d0"}), stroke: new ol.style.Stroke({color: "#111820", width: 4})})})});
        map.addLayer(badLayer);
        addLegend(`plan-unconnected-${index}`, "Не подключены", "#ff3b3b", badLayer, group);
        built.set(index, {group, routeLayer, points, features, colorOf});
    }

    function stat(parent, label, value, wide, tone) {
        const cell = element("div", (wide ? "plan-stat wide" : "plan-stat") + (tone ? " " + tone : ""));
        cell.append(element("span", "plan-stat-label", label), element("strong", "plan-stat-value", value));
        parent.append(cell);
    }

    function select(index, targetId) {
        selectedTarget = targetId === null ? null : String(targetId);
        built.get(index).routeLayer.changed();
        summaryElement.querySelectorAll(".plan-item").forEach(item => {
            item.classList.toggle("active", item.dataset.target === selectedTarget);
        });
    }

    function branchItem(index, b, colorOf) {
        const item = element("button", "plan-item");
        item.type = "button";
        item.dataset.target = String(b.target_id);
        const swatch = element("span", "swatch");
        swatch.style.backgroundColor = colorOf(b.target_id);
        const text = element("span", "plan-item-text");
        text.append(element("strong", "", `Точка ${b.target_id} · ${b.flow_tph} т/ч`),
            element("span", "plan-item-meta", `${diameterRange(b)} · ${b.length.toLocaleString("ru-RU")} м · ${shortMoney(b.cost)}`),
            element("span", "plan-item-meta", `Подключение: ${b.connection}`));
        item.append(swatch, text);
        item.addEventListener("click", () => {
            select(index, b.target_id);
            const own = built.get(index).features.filter(f => String(f.get("target_id")) === String(b.target_id));
            const extent = ol.extent.createEmpty();
            own.forEach(f => ol.extent.extend(extent, f.getGeometry().getExtent()));
            if (!ol.extent.isEmpty(extent)) view.fit(extent, {padding: [70, 70, 70, 70], maxZoom: 18, duration: 350});
        });
        return item;
    }

    function render(number) {
        summaryElement.replaceChildren(element("h2", "", `РЕШЕНИЕ НОМЕР ${number}`));
        if (state === "loading") {
            summaryElement.append(element("p", "plan-note", "Идёт расчёт вариантов. Первый расчёт файла занимает несколько минут, повторный быстрее."));
            return;
        }
        if (state === "error") {
            summaryElement.append(element("p", "plan-note error", error));
            return;
        }
        if (state !== "ready") return;
        const index = number - 1;
        const variant = variants[index];
        if (!variant) {
            summaryElement.append(element("p", "plan-note",
                "Существенно отличающегося варианта для этого набора данных не найдено: остальные стратегии дали почти ту же сеть."));
            return;
        }
        summaryElement.append(element("p", "plan-note", variant.strategy + "."));
        const stats = element("div", "plan-stats");
        stat(stats, "Подключено", `${variant.connected} из ${variant.targets}`);
        stat(stats, "Длина труб", `${variant.new_network_length.toLocaleString("ru-RU", {maximumFractionDigits: 0})} м`);
        stat(stats, "Строительство", shortMoney(variant.construction_cost));
        stat(stats, "Штраф", shortMoney(variant.unconnected_penalty));
        stat(stats, "Итоговая стоимость", shortMoney(variant.calculated_cost), true);
        stat(stats, "Показатель S", String(variant.score), true);
        summaryElement.append(stats);
        summaryElement.append(element("h3", "plan-heading", "Проверка результата"));
        const check = element("div", "plan-stats");
        stat(check, "Нарушения правил", variant.rule_violations === 0 ? "нет" : String(variant.rule_violations), false, variant.rule_violations === 0 ? "ok" : "bad");
        stat(check, "Структура сети", variant.structure_violations === 0 ? "дерево" : `${variant.structure_violations} замечаний`, false, variant.structure_violations === 0 ? "ok" : "warn");
        summaryElement.append(check);
        if (variant.violations.length) {
            const details = element("details", "plan-violations");
            details.append(element("summary", "", "Что именно найдено"));
            const list = element("ul", "warnings");
            variant.violations.forEach(x => list.append(element("li", "", x.message)));
            details.append(list);
            summaryElement.append(details);
        }
        summaryElement.append(element("h3", "plan-heading", "Подключённые точки"));
        const list = element("div", "plan-list");
        variant.branches.forEach(b => list.append(branchItem(index, b, built.get(index).colorOf)));
        summaryElement.append(list);
        if (variant.unconnected.length) {
            summaryElement.append(element("h3", "plan-heading bad", "Не удалось подключить"));
            const bad = element("div", "plan-list");
            variant.unconnected.forEach((entry, i) => {
                const item = element("button", "plan-item unconnected");
                item.type = "button";
                const swatch = element("span", "swatch");
                swatch.style.backgroundColor = "#ff3b3b";
                const text = element("span", "plan-item-text");
                text.append(element("strong", "", `Точка ${entry.id} · ${entry.flow_tph} т/ч`), element("span", "plan-item-meta", entry.reason));
                item.append(swatch, text);
                item.addEventListener("click", () => {
                    select(index, null);
                    view.animate({center: built.get(index).points[i].getGeometry().getCoordinates(), zoom: 18, duration: 350});
                });
                bad.append(item);
            });
            summaryElement.append(bad);
        }
    }

    async function load() {
        state = "loading";
        show();
        try {
            const body = new FormData();
            body.append("file", file);
            const response = await fetch("/api/plan", {method: "POST", body});
            if (!response.ok) throw new Error(await response.text());
            variants = (await response.json()).variants;
            variants.forEach((_, index) => build(index));
            state = "ready";
            enableExport();
        } catch (e) {
            state = "error";
            error = "Расчёт не выполнен: " + e.message;
        }
        fitted = false;
        show();
    }

    // Выгрузка всех вариантов в GeoJSON по разделу 7 приложения.
    function enableExport() {
        const button = document.getElementById("exportButton");
        button.disabled = false;
        button.title = "Выгрузить все варианты в GeoJSON";
        button.onclick = async () => {
            button.disabled = true;
            try {
                const response = await fetch("/api/plan/export");
                if (!response.ok) throw new Error(await response.text());
                const url = URL.createObjectURL(await response.blob());
                const link = document.createElement("a");
                link.href = url;
                link.download = "variants_2d.geojson";
                document.body.append(link);
                link.click();
                link.remove();
                setTimeout(() => URL.revokeObjectURL(url), 1000);
            } catch (e) {
                alert("Не удалось выгрузить данные: " + e.message);
            } finally {
                button.disabled = false;
            }
        };
    }

    let fitted = false;
    function show() {
        selectedTarget = null;
        const index = currentSolution === null ? -1 : currentSolution - 1;
        for (const [i, entry] of built) {
            entry.group.forEach(({layer, row}) => {
                const visible = i === index;
                layer.setVisible(visible);
                row.hidden = !visible;
                row.querySelector("input").checked = true;
            });
        }
        if (currentSolution !== null && state === "ready" && built.has(index) && !fitted) {
            fitted = true;
            const extent = ol.extent.createEmpty();
            built.get(index).group.forEach(({layer}) => ol.extent.extend(extent, layer.getSource().getExtent()));
            if (!ol.extent.isEmpty(extent)) {
                const wide = map.getSize()[0] >= 650;
                view.fit(extent, {padding: wide ? [50, 270, 50, 70] : [200, 30, 50, 30], maxZoom: 18});
            }
        }
        if (currentSolution !== null) render(currentSolution);
    }

    return function sync(solutionNumber) {
        currentSolution = solutionNumber;
        if (solutionNumber !== null && state === "idle") load();
        else show();
    };
};