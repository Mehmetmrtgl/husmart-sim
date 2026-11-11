# Beytepe "Yeni yol" v2 — Planning Simulator Sürüş Testleri

**Tarih:** 5 Kasım 2026
**Harita:** `~/autoware_map/beytepe_yeni_v2` (kaynak: `Beytepe Kampüs /Yeni yol/02_Autoware_Harita/02.10.2026_v2`, değiştirilmedi)
**Autoware:** Docker `ghcr.io/autowarefoundation/autoware:universe-cuda` (Humble, build 13.04.2026, digest `sha256:b7490c70…`)
**Araç / sensör:** sample_vehicle / sample_sensor_kit, planning simulator (algılama ve lokalizasyon simüle)
**DDS:** CycloneDDS, yalnız localhost (`~/autoware/cyclonedds.xml`)

## Sonuç

7 testin 7'si iki ayrı turda da geçti. İki turun değerleri birbirine çok yakın: sapmada fark 1 cm'den az, sürede 0.2 s'den az.
Çarpışma testi yapılmadı, çünkü bu testlerde NPC yok.

| Test | Sonuç (tur1 / tur2) | Rota [m] | Süre [s] | Maks sapma [m] | RMS sapma [m] | Maks yavaşlama [m/s²] | Hedefe uzaklık [m] |
|---|---|---|---|---|---|---|---|
| Düz şerit takibi (doğu yolu) | PASS / PASS | 409.4 | 66.9 / 66.9 | 0.025 / 0.024 | 0.007 / 0.007 | 1.1 / 1.11 | 0.03 / 0.01 |
| Kavşakta sola dönüş (GB kavşağı) | PASS / PASS | 227.9 | 41.2 / 41.4 | 0.163 / 0.164 | 0.059 / 0.059 | 1.12 / 1.12 | 0.02 / 0.04 |
| Kavşakta sağa dönüş | PASS / PASS | 227.9 | 45.2 / 45.0 | 0.102 / 0.1 | 0.041 / 0.042 | 1.12 / 1.12 | 0.03 / 0.02 |
| Güney → kuzey yol bağlantısı | PASS / PASS | 919.6 | 92.9 / 92.7 | 0.165 / 0.164 | 0.043 / 0.044 | 1.14 / 1.14 | 0.02 / 0.0 |
| Şerit değiştirme (sağ → sol)* | PASS / PASS | 146.9 | 37.0 / 36.8 | 2.274 / 2.276 | 0.494 / 0.494 | 1.13 / 1.12 | 0.02 / 0.01 |
| Uzun rota planlama güney → kuzey uç (sürüşsüz) | PASS / PASS | 3848.3 | – | – | – | – | – |
| Uzun rota planlama kuzey uç → güney (sürüşsüz) | PASS / PASS | 3062.1 | – | – | – | – | – |

- "Rota" sütunu rotadaki lanelet'lerin toplam uzunluğudur. Aracın gerçekte sürdüğü mesafe bundan kısadır.
- Test 2 ile Test 3'ün rota uzunluğu tesadüfen aynı çıktı (ikisi de 228.0 m). Rotalar farklı, bu haritadan ayrıca doğrulandı.
- \* Şerit değiştirmede "sapma", aracın en yakın şerit merkezine uzaklığıdır. Bu iki şeridin merkezleri 4.46 m aralıklı olduğundan, şerit değişiminin ortasında sapma ≈2.27 m çıkması beklenen bir durumdur.
  - İlk değerlendirmede bu test için 2.2 m'lik hatalı bir eşik kullanıldı ve test "FAIL" göründü. Eşik 2.6 m'ye (aralığın yarısı + 0.35 m) düzeltildi.
  - Ayrı ölçüm: şerit değişimi ≈6.5 s sürdü. Araç hedef şerit merkezini en fazla ≈0.3 m aştı, sonra şeritte kaldı.

**Ölçütler:** hedefe varmak (`/api/routing/state = ARRIVED`), hedefe uzaklığın 1.5 m'den az olması, yanal sapmanın 0.5 m'den az olması, en büyük yavaşlamanın 3 m/s²'den az olması (konfor sınırı) ve MRM tetiklenmemesi. Ayrıca araç 30 s'den uzun süre takılı kalırsa test kalır.
Hız, Autoware'in varsayılan sınırı olan 15 km/h ile sınırlı. Daha yüksek hızlar test edilmedi.

## Bulgular

### 1. FastDDS ile behavior planner çöküyor → CycloneDDS'e geçildi (düzeltildi)
- **Belirti:** İlk çalıştırmada imajın varsayılan katmanı olan FastDDS kullanıldı. Bu sırada `/planning/trajectory` yayın hızı defalarca düştü ve her seferinde 1 s'den kısa süren `EMERGENCY_STOP` (MRM) darbeleri geldi.
- **Çökme:** İkinci rota verilirken `behavior_planning_container` segfault verdi (exit −11) ve yeniden başlamadı. Bundan sonra otonom moda geçiş reddedildi ("target mode is not available").
- **Kök neden:** Yığın izi `rmw_fastrtps_shared_cpp::__rmw_wait` içinde. Yani hata DDS katmanında, harita ya da Autoware modülleri değil.
- **Çözüm:** Simülatör CycloneDDS ile yeniden başlatıldı. Bu, Autoware'in önerdiği katman ve önceki kurulumda da kullanılıyordu.
- **Sonuç:** CycloneDDS ile iki tam test turunda (≈20 dk) sıfır çökme, sıfır MRM ve sıfır trajectory hız düşüşü görüldü.

### 2. v2'de güney–kuzey bağlantısı çalışıyor
v1 README'sinde kuzey yola (≈2.4 km) rota oluşmadığı yazıyordu. v2'de bu sorun yok:
- Güneydeki 236 yol şeridinin 220'sinden kuzey yola rota çıkıyor.
- Kuzeydeki 48 şeridin tamamından güneye dönüş rotası çıkıyor.
- Test 4'te araç bağlantı kavşağından geçip kuzey yola sürdü.
- Test 6a ve 6b'de 3.8 km ve 3.1 km'lik uçtan uca rotalar planlandı.

### 3. Harita kalitesi (sürüşü etkilemedi, düzeltilmesi önerilir)
- **Kendini kesen lanelet:** 2000268, (−180.5, 164.6) civarında. Poligonu (sol sınır + sağ sınır) kendini kesiyor.
- **Triangülasyon uyarısı:** `planning_validator` sürekli `Could not find valid vertex for ear clipping triangulation` uyarısı veriyor (≈2800 kez / 10 dk). Uyarı, test rotaları 2000268'den geçmediği halde de geliyor, yani bu tek lanelet'ten fazlası var. Sürüşü etkilemedi.
- **Çıkmaz uçlar:** 7 kısa lanelet ardılı olmayan çıkmaz uç: 2000281, 2000119, 2000056 (≈(−213, −296), 3.7 m) ve 2000109, 2000106, 2000003, 2000136 (≈(−100…−141, 165…183)).
- **Öncülsüz lanelet'ler:** 6 lanelet'in öncülü yok. Bunlar büyük olasılıkla RoadRunner kavşak kesimlerinden kalan parçalar.
- **Şerit genişliği:** yol şeritlerinde medyan 3.50 m. Doğu yolundaki iki şerit 4.25 m ve 4.83 m.

### 4. Önceki rapordaki sorunların durumu
- **Pull-out (ClothoidPullOut) sorunu:** Görülmedi. Bütün başlangıçlar düz şerit ortasına verildi.
- **`projector_type`:** v2'deki `Local` yazımı bu Autoware sürümünde doğru. Eski `local` yazımı artık "deprecated" uyarısı veriyor.
- **PCD:** 4.112.441 noktanın tamamı yüklendi. Lanelet `ele` değerleriyle PCD yüzeyi örtüşüyor (medyan fark 0.00 m, %90 aralık ±0.12 m).

## Kapsam dışı / sınırlamalar
- Planning simulator: lokalizasyon ve algılama simüle ediliyor. NDT, PCD ile lokalizasyon ve sensör gürültüsü test edilmedi.
- NPC'li senaryolar (takip, fren, kesme, yaya) bu turda yok. Önceki rapordaki scenario_simulator_v2 testleri bu imajla tekrar edilmedi.
- RViz videosu alınmadı, çünkü `:21` ekranı testler sırasında kilitliydi. Grafikler kayıtlı kinematik veriden üretildi.

## Dosyalar
| Yol | İçerik |
|---|---|
| `~/autoware/run_planning_sim.sh` | Simülatörü başlatır (Docker + CycloneDDS) |
| `~/autoware/cyclonedds.xml` | DDS yapılandırması |
| `analysis/run_tests.py`, `analysis/specs.json` | Test koşucusu ve test tanımları (konteyner içinde çalışır) |
| `analysis/map_graph.py`, `map_graph.json` | Routing graph analizi |
| `analysis/polygon_check.py`, `polygon_check.json` | Lanelet poligon kontrolü |
| `analysis/make_plots.py` | Grafik ve tablo üretimi |
| `results_run1/`, `results_run2/` | Her test için ham JSON (zaman serileri dahil) |
| `report_run1/`, `report_run2/` | Grafikler (`00_genel_harita.png` + test başına bir PNG) |

**Tekrar çalıştırma:**
```bash
~/autoware/run_planning_sim.sh        # simülatörü başlat (~2 dk hazır olur)
cd ~/autoware/beytepe_tests_v2
docker exec autoware_planning_sim mkdir -p /tmp/bt
docker cp analysis/run_tests.py autoware_planning_sim:/tmp/bt/
docker cp analysis/specs.json  autoware_planning_sim:/tmp/bt/
docker exec autoware_planning_sim bash -c 'source /opt/autoware/setup.bash; cd /tmp/bt; python3 run_tests.py specs.json /tmp/bt/results'
docker cp autoware_planning_sim:/tmp/bt/results ./results_yeni
python3 analysis/make_plots.py results_yeni analysis/map_graph.json report_yeni
```
