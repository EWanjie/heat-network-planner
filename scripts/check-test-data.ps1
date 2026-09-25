# Проверка тестовых файлов через работающий сервис: код ответа /api/upload и нужное замечание в диагностике.
# Запуск: powershell -File scripts\check-test-data.ps1 [-Base http://localhost:8080]
param([string]$Base = "http://localhost:8080")
$dir = Join-Path (Split-Path $PSScriptRoot -Parent) "test-data"
# файл = ожидаемый код ; фрагмент замечания (необязательно)
$expected = [ordered]@{
    "valid.geojson"                          = @(200, $null)
    "synthetic_full.geojson"                 = @(200, $null)
    "ok_id_types.geojson"                    = @(200, $null)
    "warn_chamber_5_segments.geojson"        = @(200, "примыкает больше 4")
    "warn_chamber_off_network.geojson"       = @(200, "не лежат на существующей сети")
    "warn_diameter_shrinks.geojson"          = @(200, "Диаметр уменьшается")
    "warn_segment_disconnected.geojson"      = @(200, "Не связаны ни с одним источником")
    "warn_no_source.geojson"                 = @(200, "нет источника")
    "warn_unused_types_and_restrictions.geojson" = @(200, "oks_future")
    "bad_coordinates_range.geojson"          = @(400, $null)
    "bad_diameter.geojson"                   = @(400, "таблице 1")
    "bad_missing_diameter.geojson"           = @(400, "не задан diameter")
    "bad_duplicate_id.geojson"               = @(400, "Повторяющийся id")
    "bad_geometry_kind.geojson"              = @(400, "должен иметь геометрию")
    "bad_geometry_many.geojson"              = @(400, "Ошибки геометрии (3)")
    "bad_geometry_restriction.geojson"       = @(400, $null)
    "bad_json_duplicate_key.geojson"         = @(400, "не является корректным JSON")
    "bad_json_not_json.geojson"              = @(400, "не является корректным JSON")
    "bad_json_truncated.geojson"             = @(400, "не является корректным JSON")
    "bad_missing_flow.geojson"               = @(400, "flow_tph")
    "bad_no_geometry.geojson"                = @(400, "Нет geometry")
    "bad_polygon_unclosed.geojson"           = @(400, "не замкнуто")
    "bad_restriction_point.geojson"          = @(400, "LineString, MultiLineString, Polygon или MultiPolygon")
}
$fail = 0
foreach ($name in $expected.Keys) {
    $path = Join-Path $dir $name
    $out = [IO.Path]::GetTempFileName()
    $code = & curl.exe -s -o $out -w "%{http_code}" -F "file=@$path" "$Base/api/upload"
    $body = [IO.File]::ReadAllText($out, [Text.Encoding]::UTF8); [IO.File]::Delete($out)
    $want, $text = $expected[$name]
    $ok = ($code -eq "$want") -and (-not $text -or $body.Contains($text))
    if (-not $ok) { $fail++ }
    "{0,-46} {1} (ожидалось {2}){3}" -f $name, $code, $want, $(if ($ok) { "" } else { "  <-- НЕ СОВПАЛО: $($body.Substring(0,[Math]::Min(160,$body.Length)))" })
}
"расхождений: $fail из $($expected.Count)"
exit $fail