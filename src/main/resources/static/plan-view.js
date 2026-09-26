// Результат расчёта на вкладках решений: слои трасс и камер, сводка, проверка и список точек поверх общей карты.
// Вкладки 1-3 — основные варианты (строго по данным и правилам), 4-6 — дополнительные (с допущениями), включаются фильтром.
window.mountPlan = function mountPlan(map, file, legendItems, summaryElement) {
    const PALETTE = ["#ff668c", "#7fd1ff", "#ffd166", "#80f2a3", "#c792ea", "#ff9e64", "#4dd0c8", "#f78fb3", "#a3e635", "#f472b6"];
    const MAIN_TABS = 3;
    const format = new ol.format.GeoJSON();
    const view = map.getView();
    const shortMoney = value => (value / 1e6).toLocaleString("ru-RU", {maximumFractionDigits: 1}) + " млн ₽";
    const diameterRange = b => b.diameter_min === b.diameter_max ? `ДУ ${b.diameter_min}` : `ДУ ${b.diameter_min}–${b.diameter_max}`;
    let state = "idle";
    let mainVariants = [];
    let extraVariants = [];
    let diagnostics = [];
    let error = "";
    let currentSolution = null;
    let jobId = null;
    let progressText = "";
    let selectedTarget = null;
    const built = new Map();

    const keyOf = number => number <= MAIN_TABS ? `v${number - 1}` : `a${number - MAIN_TABS - 1}`;
    const variantOf = number => number <= MAIN_TABS ? mainVariants[number - 1] : extraVariants[number - MAIN_TABS - 1];

    function element(tag, className, text) {
        const node = document.createElement(tag);
        if (className) node.className = className;
        if (text !== undefined) node.textContent = text;
        return node;
    }

    function addLegend(key, title, color, layer, group, optional) {
        const row = element("label", "legend-row");
        row.dataset.layer = key;
        row.hidden = true;
        const check = document.createElement("input");
        check.type = "checkbox";
        check.checked = !optional;
        check.setAttribute("aria-label", title);
        check.addEventListener("change", () => layer.setVisible(check.checked));
        const swatch = element("span", "swatch");
        swatch.style.backgroundColor = color;
        row.append(check, swatch, element("span", "", title));
        legendItems.append(row);
        group.push({layer, row, optional: Boolean(optional)});
    }

    function build(key, variant) {
        const group = [];
        const features = format.readFeatures(variant.routes, {dataProjection: "EPSG:4326", featureProjection: "EPSG:3857"});
        const ids = variant.branches.map(b => String(b.target_id));
        const colorOf = id => PALETTE[Math.max(0, ids.indexOf(String(id))) % PALETTE.length];
        const casing = width => new ol.style.Style({stroke: new ol.style.Stroke({color: "#0d1117", width: width + 3.5, lineCap: "round"})});
        const cache = new Map();
        const routeLayer = new ol.layer.Vector({
            source: new ol.source.Vector({features, wrapX: false}), zIndex: 450, visible: false,
            style: feature => {
                const id = String(feature.get("target_id"));
                const width = Math.min(9, 3 + feature.get("diameter") / 60);
                const styleKey = `${id}:${width}:${id === selectedTarget}:${feature.get("laying_method")}`;
                if (!cache.has(styleKey)) {
                    const color = colorOf(id);
                    const styles = [casing(width), new ol.style.Style({stroke: new ol.style.Stroke({
                        color, width, lineCap: "round", lineDash: feature.get("laying_method") === "special" ? [10, 8] : undefined})})];
                    if (id === selectedTarget) styles.unshift(new ol.style.Style({stroke: new ol.style.Stroke({color: "#ffffff", width: width + 8, lineCap: "round"})}));
                    cache.set(styleKey, styles);
                }
                return cache.get(styleKey);
            }});
        map.addLayer(routeLayer);
        addLegend(`plan-routes-${key}`, "Новые трассы (толщина — ДУ)", "#ff668c", routeLayer, group);
        const chamberLayer = new ol.layer.Vector({
            source: new ol.source.Vector({wrapX: false, features: format.readFeatures(variant.chambers,
                {dataProjection: "EPSG:4326", featureProjection: "EPSG:3857"})}), zIndex: 550, visible: false,
            style: new ol.style.Style({image: new ol.style.RegularShape({points: 4, radius: 8, angle: Math.PI / 4,
                fill: new ol.style.Fill({color: "#ffffff"}), stroke: new ol.style.Stroke({color: "#111820", width: 2})})})});
        map.addLayer(chamberLayer);
        addLegend(`plan-chambers-${key}`, "Новые камеры и врезки", "#ffffff", chamberLayer, group);
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
        addLegend(`plan-unconnected-${key}`, "Не подключены", "#ff3b3b", badLayer, group);
        built.set(key, {group, routeLayer, points, features, colorOf});
    }

    function stat(parent, label, value, wide, tone) {
        const cell = element("div", (wide ? "plan-stat wide" : "plan-stat") + (tone ? " " + tone : ""));
        cell.append(element("span", "plan-stat-label", label), element("strong", "plan-stat-value", value));
        parent.append(cell);
    }

    function select(key, targetId) {
        selectedTarget = targetId === null ? null : String(targetId);
        built.get(key).routeLayer.changed();
        summaryElement.querySelectorAll(".plan-item").forEach(item => {
            item.classList.toggle("active", item.dataset.target === selectedTarget);
        });
    }

    function branchItem(key, b, colorOf) {
        const item = element("button", "plan-item");
        item.type = "button";
        item.dataset.target = String(b.target_id);
        const swatch = element("span", "swatch");
        swatch.style.backgroundColor = colorOf(b.target_id);
        const text = element("span", "plan-item-text");
        text.append(element("strong", "", `Точка ${b.target_id} · ${b.flow_tph} т/ч`),
            element("span", "plan-item-meta", `${diameterRange(b)} · ${b.length.toLocaleString("ru-RU")} м · ${shortMoney(b.cost)}`),
            element("span", "plan-item-meta", `Подключение: ${b.connection}${b.fallback_approach ? " · запасной подход к зданию" : ""}`));
        item.append(swatch, text);
        item.addEventListener("click", () => {
            select(key, b.target_id);
            const own = built.get(key).features.filter(f => String(f.get("target_id")) === String(b.target_id));
            const extent = ol.extent.createEmpty();
            own.forEach(f => ol.extent.extend(extent, f.getGeometry().getExtent()));
            if (!ol.extent.isEmpty(extent)) view.fit(extent, {padding: [70, 70, 70, 70], maxZoom: 18, duration: 350});
        });
        return item;
    }

    function render(number) {
        const additional = number > MAIN_TABS && number <= 2 * MAIN_TABS;
        summaryElement.replaceChildren(element("h2", "", additional
            ?`ДОПОЛНИТЕЛЬНЫЙ ВАРИАНТ ${number - MAIN_TABS}` : `РЕШЕНИЕ НОМЕР ${number}`));
        if (state === "loading") {
            summaryElement.append(element("p", "plan-note", progressText || "Загружаем дороги и ставим расчёт в очередь."));
            summaryElement.append(element("p", "plan-note", "Первый расчёт файла занимает около трёх минут, повторный быстрее. Страницу можно не закрывать: расчёт идёт на сервере."));
            return;
        }
        if (state === "error") {
            summaryElement.append(element("p", "plan-note error", error));
            return;
        }
        if (state !== "ready") return;
        const key = keyOf(number);
        const variant = variantOf(number);
        if (!variant) {
            summaryElement.append(element("p", "plan-note", additional
                ? `Для решения ${number - MAIN_TABS} допущения не понадобились: дополнительного варианта с той же логикой нет.`
                : "Существенно отличающегося варианта для этого набора данных не найдено: остальные стратегии дали почти ту же сеть."));
            return;
        }
        summaryElement.append(element("p", "plan-note", variant.strategy + "."));
        if (variant.assumptions.length) {
            const box = element("div", "plan-assumptions");
            box.append(element("strong", "", "Допущения этого варианта"));
            const list = element("ul", "warnings");
            variant.assumptions.forEach(a => list.append(element("li", "", a)));
            box.append(list);
            summaryElement.append(box);
        }
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
        variant.branches.forEach(b => list.append(branchItem(key, b, built.get(key).colorOf)));
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
                    select(key, null);
                    view.animate({center: built.get(key).points[i].getGeometry().getCoordinates(), zoom: 18, duration: 350});
                });
                bad.append(item);
            });
            summaryElement.append(bad);
        }
        if (diagnostics.length) {
            const details = element("details", "plan-violations");
            details.append(element("summary", "", "Замечания к расчёту"));
            const notes = element("ul", "warnings");
            diagnostics.forEach(x => notes.append(element("li", "", x)));
            details.append(notes);
            summaryElement.append(details);
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
            jobId = (await response.json()).jobId;
            let result = null;
            while (!result) {
                await new Promise(resolve => setTimeout(resolve, 2000));
                const poll = await fetch(`/api/plan/${jobId}`);
                if (!poll.ok) throw new Error(await poll.text());
                const status = await poll.json();
                if (status.status === "FAILED") throw new Error(status.message);
                if (status.status === "DONE") result = status.result;
                else {
                    progressText = status.status === "QUEUED"
                        ? `В очереди: впереди заданий ${Math.max(0, status.queuePosition - 1)}.`
                        : `${status.message} (${status.progress} %).`;
                    if (currentSolution !== null) render(currentSolution);
                }
            }
            mainVariants = result.variants;
            extraVariants = result.additional || [];
            diagnostics = result.diagnostics || [];
            mainVariants.forEach((v, i) => build(`v${i}`, v));
            extraVariants.forEach((v, i) => { if (v) build(`a${i}`, v); });
            state = "ready";
            enableExport();
        } catch (e) {
            state = "error";
            error = "Расчёт не выполнен: " + e.message;
        }
        fitted = false;
        show();
    }

    // Выгрузка в GeoJSON по разделу 7 приложения: основные варианты, а с включённым фильтром и дополнительные.
    function enableExport() {
        const button = document.getElementById("exportButton");
        button.disabled = false;
        button.title = "Выгрузить варианты в GeoJSON";
        button.onclick = async () => {
            button.disabled = true;
            try {
                const extra = document.getElementById("additionalToggle").checked;
                const response = await fetch(`/api/plan/${jobId}/export?additional=${extra}`);
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
        const key = currentSolution === null ? null : keyOf(currentSolution);
        for (const [k, entry] of built) {
            entry.group.forEach(({layer, row, optional}) => {
                const visible = k === key;
                row.querySelector("input").checked = !optional;
                layer.setVisible(visible && !optional);
                row.hidden = !visible;
            });
        }
        if (currentSolution !== null && state === "ready" && built.has(key) && !fitted) {
            fitted = true;
            const extent = ol.extent.createEmpty();
            built.get(key).group.forEach(({layer}) => ol.extent.extend(extent, layer.getSource().getExtent()));
            if (!ol.extent.isEmpty(extent)) {
                const wide = map.getSize()[0] >= 650;
                view.fit(extent, {padding: wide ? [50, 270, 50, 70] : [200, 30, 50, 30], maxZoom: 18});
            }
        }
        if (currentSolution !== null) render(currentSolution);
    }

    // Расчёт занимает минуты, поэтому он стартует сразу после открытия страницы результатов, пока пользователь
    // смотрит исходные данные; к моменту перехода на вкладку решения он уже идёт или закончен.
    setTimeout(() => { if (state === "idle") load(); }, 800);

    return function sync(solutionNumber) {
        currentSolution = solutionNumber;
        if (solutionNumber !== null && state === "idle") load();
        else show();
    };
};