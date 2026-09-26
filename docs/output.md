# Выходные данные

Результат каждого режима — один GeoJSON `FeatureCollection` в WGS 84 (раздел 7 приложения); варианты различаются по `variant_id`
(`v1`, `v2`, `v3` — основные, `a1`, `a2`, `a3` — дополнительные, выгружаются при `additional=true`).
Получение: `GET /api/plan/{jobId}/export`, кнопка «Выгрузить данные» в интерфейсе.

| object_type | Геометрия | Основные атрибуты |
|---|---|---|
| `heat_network` | LineString | `id`, `variant_id`, `start_node_id`, `end_node_id`, `flow_tph`, `diameter`, `length`, `laying_method` (`base`/`special`), `depth_start`/`depth_end` (`null` в 2D), `cost` |
| `heat_chamber` | Point | `id`, `variant_id`, `diameter` (наибольший ДУ примыкающих участков), `cost` — только **новые** камеры |
| `technical_node` | Point | `id`, `variant_id` — где без ветвления меняется ДУ или способ прокладки |
| `variant_summary` | нет | `rank`, `construction_cost`, `chamber_construction_cost`, `existing_chamber_tie_in_count`, `existing_chamber_tie_in_cost`, `unconnected_penalty`, `calculated_cost`, `new_network_length`, `score`, `unconnected_oks_ids` |

- Начало и конец `heat_network` совпадают с узлами, на которые ссылаются `start_node_id` и `end_node_id`: точка подключения, существующая
  или новая камера, технический узел. Повороты остаются внутренними вершинами линии.
- Тип идентификатора сохраняется (число остаётся числом, строка — строкой).
- Стоимости округлены до копеек, итоговые значения `variant_summary` посчитаны по записанным значениям.
- У дополнительных вариантов в сводке есть дополнительное свойство `assumptions` (перечень допущений); дополнительные свойства допустимы приложением.