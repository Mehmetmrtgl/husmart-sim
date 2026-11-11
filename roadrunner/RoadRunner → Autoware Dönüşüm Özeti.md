# RoadRunner → Autoware Dönüşüm Özeti

**Tarih:** 13 Ekim 2026\
**Proje:** Beytepe Kampüsü HD Harita Dönüşümü (BEYTEPE_21_09_26)

---

## Amaç

RoadRunner'da oluşturulmuş Beytepe kampüsü yol ağını Autoware'de kullanılabilir hale getirmek. Hedef: otonom araç simülasyonu için gerekli üç dosyayı üretmek.

---

## Kaynak Dosyalar

| Dosya | Format | İçerik |
| --- | --- | --- |
| BEYTEPE_21_09_26.xodr | OpenDRIVE XML | 167 yol, 14 kavşak, projeksiyon bilgisi |
| BEYTEPE_21_09_26.geojson | GeoJSON | 1835 özellik: şeritler, sınırlar, kavşaklar |
| BEYTEPE_21_09_26.fbx | Kaydara FBX | 3D görsel mesh (dönüşümde kullanılmadı) |
| BEYTEPE_21_09_26.rrscene | Binary | RoadRunner sahnesi (okunamaz format) |

---

## Üretilen Çıktı Dosyaları

### 1. `lanelet2_map.osm`

Autoware'in yol ağını okuduğu temel harita dosyası.

**Final içerik (v4):**

- **319 road lanelet** — araç şeritleri
- **173 walkway lanelet** — yaya yolları
- **6 crosswalk lanelet** — yaya geçitleri
- **843 way** — şerit sınır çizgileri
  - 141 `dashed` (şerit değiştirme serbest)
  - 702 `solid` (şerit değiştirme yasak)
- **14.420 node** — her biri `local_x`, `local_y`, `ele` tag'leriyle

**Projeksiyon:** Local Transverse Mercator, origin = (39.871164°N, 32.735689°E)

### 2. `map_projector_info.yaml`

Autoware'e koordinat sistemini tanıtan yapılandırma dosyası.

```yaml
projector_type: local
map_origin:
  latitude: 39.87116440956932
  longitude: 32.7356890084472
  altitude: 0.0
```

### 3. `pointcloud_map.pcd`

LiDAR nokta bulutu haritası. **Önemli:** Gerçek LiDAR verisi olmadığı için GeoJSON geometrisinden sentez üretildi (173.462 nokta, 0.5 m aralıklı). Gerçek lokalizasyon için sahadan LiDAR taraması gerekir.

---

## Karşılaşılan Hatalar ve Çözümler

| # | Hata | Kök Neden | Uygulanan Çözüm |
| --- | --- | --- | --- |
| 1 | RViz: `Contains invalid floating point values (nans or infs)` — tüm lanelet ID'lerinde | `local` projektör ile Autoware node'ların `lat`/`lon` attribute'ını değil, `local_x`/`local_y` tag'lerini okuyor; ilk OSM'de bu tag'ler yoktu | Her node'a `local_x`, `local_y`, `ele` tag'leri eklendi; tüm ID'ler negatif yapıldı (Lanelet2 convention) |
| 2 | `map_projector_info.yaml` okunmuyor | YAML'a yanlışlıkla ROS 2 param wrapper (`/**:\n ros__parameters:`) eklenmişti | Düz format'a döndürüldü: `projector_type: local` başta |
| 3 | `Route API is not ready` — rota planlanamıyor | Predecessor/successor bağlantıları OSM tag olarak yazılmıştı; Lanelet2 routing graph builder bu tag'leri okumaz, yalnızca **paylaşılan endpoint node**'larına bakarak topolojiyi çıkarır | Node deduplication hassasiyeti 1 cm'den 1 mm'ye indirildi; ortak uç noktalar aynı node ID'sini paylaşır hale geldi → 181 longitudinal bağlantı |
| 4 | RViz'de `center_line_line/0` ve bazı lanelet ID'lerinde hâlâ NaN | 135 way'de ardışık aynı node ID'leri vardı (rounding collision); sıfır uzunluklu lanelet'lerde centerline hesaplayıcı sıfıra bölüyordu | Tüm ardışık duplicate node'lar kaldırıldı; 0.5 m altındaki 38 dejenere kavşak lanelet'i silindi |
| 5 | `ClothoidPullOut: Circular path generation failed` — araç hareket etmiyor | `start_planner` pull-out için en az \~5 m düz yol arıyor; 2D Pose Estimate kavşak yakınına verilmişti | **Harita kaynaklı değil** — config sorunu. Çözüm: 2D Pose Estimate'i düz ve uzun bir şerit üzerine vermek; ya da `enable_clothoid_pull_out: false` / `enable_shift_pull_out: true` |
| 6 | Araç şerit değiştirmiyor | Tüm sınır way'leri `solid` olarak oluşturulmuştu; Autoware şerit değiştirme kararını sınırın `subtype` tag'inden alır | Aynı yönde giden iki şeridi paylaşan sınırlar `dashed` yapıldı (141 way); `lane_change: yes` tag'i 260 lanelet'e eklendi |
| 7 | Haritada yaya geçitleri görünmüyor | GeoJSON'daki `Type=Crosswalk` polygon'ları hiç işlenmemişti | 6 crosswalk polygon'u lanelet'e dönüştürüldü: `subtype: crosswalk`, `participant:pedestrian: yes` |

---

## Mevcut Durum

| Bileşen | Durum |
| --- | --- |
| Harita yükleme (lanelet2_map_loader) | ✅ Çalışıyor |
| Lokalizasyon (Localization \| Initialized) | ✅ Çalışıyor |
| Rota planlama (Routing \| Set) | ✅ Çalışıyor |
| Şerit değiştirme | ✅ Düzeltildi (v4) |
| Yaya geçitleri haritada | ✅ Düzeltildi (v4) |
| Araç hareketi | ⚠️ Test gerekiyor |
| LiDAR / Nokta bulutu | ⚠️ Sentez PCD — gerçek tarama yok |
| Kasişler (speed bump) | ❌ RoadRunner'da modellenememiş — kaynak veride yok |

---

## Kalan Açık Sorunlar

**1. Gerçek LiDAR nokta bulutu**\
`ndt_scan_matching` tabanlı lokalizasyon sentez PCD ile çalışmaz. Kampüste gerçek LiDAR taraması yapılarak `pointcloud_map.pcd` yeniden oluşturulmalı.

**2. Kasişler (speed bump)**\
xodr ve GeoJSON'da speed bump objesi yok. RoadRunner'da yeniden modellenip dışa aktarılması gerekiyor. Autoware'de `speed_bump` regulatory element olarak eklenebilir.

**3. `ClothoidPullOut` sorunu**\
Araç hareketi hâlâ doğrulanmadı. 2D Pose Estimate'i kavşak dışında düz bir şeride vermek veya `start_planner.param.yaml`'da `enable_shift_pull_out: true` ayarlamak gerekebilir.

**4. Trafik işaretleri ve sinyal regulatory element'leri**\
Mevcut haritada trafik ışığı, dur işareti gibi regulatory element'ler yok. RoadRunner'da trafik nesneleri tanımlanmışsa bunların OSM relation olarak eklenmesi gerekir.