/* Minimal adapter: existing tabs, map and export button display verified solver output. */
function renderCalculatedSummary(container, variant) {
    const summary = variant.features.find(f => f.properties?.object_type === "variant_summary")?.properties;
    if (!summary) return;
    const title = document.createElement("h2"); title.textContent = `Решение ${summary.rank}`; container.append(title);
    const list = document.createElement("ul"); list.className = "counts";
    const number = value => Number(value).toLocaleString("ru-RU", {maximumFractionDigits: 2});
    for (const [label, value] of [["Строительство", `${number(summary.construction_cost)} ₽`],
        ["Новая сеть", `${number(summary.new_network_length)} м`], ["Камеры", `${number(summary.chamber_construction_cost)} ₽`],
        ["Врезки", summary.existing_chamber_tie_in_count], ["Не подключено", summary.unconnected_oks_ids.length],
        ["Штраф", `${number(summary.unconnected_penalty)} ₽`], ["Оценка", number(summary.score)]]) {
        const row = document.createElement("li"), name = document.createElement("span"), amount = document.createElement("strong");
        name.textContent = label; amount.textContent = value; row.append(name, amount); list.append(row);
    }
    container.append(list);
    const note = document.createElement("p"); note.textContent = variant.description; container.append(note);
}
function exportCalculatedVariant(variant) {
    const blob = new Blob([JSON.stringify(variant, null, 2)], {type: "application/geo+json"});
    const url = URL.createObjectURL(blob), link = document.createElement("a");
    const rank = variant.features.find(f => f.properties?.object_type === "variant_summary")?.properties.rank || 1;
    link.href = url; link.download = `Решение-${rank}.geojson`; link.click(); setTimeout(() => URL.revokeObjectURL(url), 1000);
}

