# AZAK Rocket 2026 Payload Firmware
## AZAK Roket 2026 Görev Yükü Yazılımı

This repository contains the payload firmware developed for the AZAK Rocket Team's 2026 rocket project.

Bu repository, AZAK Roket Takımı'nın 2026 yılı roket projesi kapsamında geliştirilen görev yükü yazılımını içermektedir.

The firmware integrates environmental sensing, GPS positioning, LoRa telemetry, local SD card logging, signal filtering, barometric altitude estimation, and descent-triggered recovery signaling within a single embedded system.

Firmware; çevresel sensör ölçümleri, GPS konum bilgisi, LoRa telemetri, SD kart veri kaydı, sinyal filtreleme, barometrik irtifa hesaplama ve düşüşe bağlı kurtarma sinyali tetikleme işlemlerini tek bir gömülü sistem içerisinde birleştirmektedir.

---

## Overview | Genel Bakış

The system is designed to operate as the rocket payload telemetry and sensing unit.

Sistem, roketin görev yükü telemetri ve sensör birimi olarak çalışacak şekilde tasarlanmıştır.

The firmware periodically collects data from multiple sensors, filters noisy measurements, calculates derived values, records telemetry to an SD card, and transmits a binary telemetry packet over LoRa.

Yazılım, birden fazla sensörden periyodik olarak veri toplar, gürültülü ölçümleri filtreler, türetilmiş değerleri hesaplar, telemetri verilerini SD karta kaydeder ve LoRa üzerinden binary telemetri paketi gönderir.

---

## Main Features | Temel Özellikler

- LoRa telemetry transmission  
  LoRa üzerinden telemetri gönderimi

- GPS position and altitude acquisition  
  GPS konum ve irtifa verilerinin alınması

- Barometric pressure measurement  
  Barometrik basınç ölçümü

- Temperature and humidity measurement  
  Sıcaklık ve nem ölçümü

- Air density calculation  
  Hava yoğunluğu hesaplama

- Barometric altitude estimation  
  Barometrik irtifa hesaplama

- Median filtering  
  Medyan filtreleme

- Exponential Moving Average filtering  
  Üstel hareketli ortalama (EMA) filtreleme

- Kalman filtering  
  Kalman filtreleme

- SD card telemetry logging  
  SD karta telemetri kaydı

- Binary telemetry packet construction  
  Binary telemetri paketi oluşturma

- Checksum-based packet verification  
  Checksum tabanlı paket doğrulama

- Relative altitude tracking  
  Göreli irtifa takibi

- Descent detection logic  
  Düşüş algılama mantığı

- Recovery signaling using siren and visual beacon  
  Siren ve Kartalgözü ile kurtarma sinyali

---

## Hardware | Kullanılan Donanımlar

The firmware interfaces with the following main hardware components:

Yazılım aşağıdaki temel donanım bileşenleriyle haberleşmektedir:

| Component | Purpose |
|---|---|
| ESP32-S3 | Main microcontroller |
| MS5611 | Barometric pressure and temperature measurement |
| SHT31 | Temperature and humidity measurement |
| GPS Module | Position and GPS altitude |
| SX1262-based LoRa Module | Wireless telemetry transmission |
| MicroSD Card | Local telemetry recording |
| Siren | Recovery / descent signaling |
| Signal Beacon | Visual recovery indication |
| Buzzer | Startup audible indication |

---

## Software Libraries | Kullanılan Kütüphaneler

```text
MS5611
Wire
Adafruit_SHT31
TinyGPS++
HardwareSerial
RadioLib
SPI
SD
cstring
math
```

---

## Sensor Processing | Sensör İşleme

### SHT31

The SHT31 is used to measure:

- Temperature
- Relative humidity

SHT31 sensörü:

- Sıcaklık
- Bağıl nem

ölçümleri için kullanılmaktadır.

Invalid readings are detected using `isnan()` checks before the data is accepted.

Geçersiz sensör okumaları `isnan()` kontrolü ile filtrelenmektedir.

### MS5611

The MS5611 is used for:

- Atmospheric pressure
- Sensor temperature
- Barometric altitude estimation

MS5611 sensörü:

- Atmosfer basıncı
- Sensör sıcaklığı
- Barometrik irtifa

hesaplamalarında kullanılmaktadır.

Pressure measurements are checked against a valid operating range before being accepted.

Basınç ölçümleri kullanılmadan önce geçerli çalışma aralığı açısından kontrol edilmektedir.

---

## Signal Filtering | Sinyal Filtreleme

Multiple filtering techniques are combined to improve telemetry stability.

Telemetri verilerinin kararlılığını artırmak amacıyla birden fazla filtreleme yöntemi birlikte kullanılmaktadır.

### Median Filter | Medyan Filtresi

A five-sample median filter is used to suppress sudden spikes in pressure and humidity measurements.

Basınç ve nem ölçümlerindeki ani sıçramaları azaltmak için 5 örnekli medyan filtresi kullanılmaktadır.

```text
MEDIAN_SIZE = 5
```

### EMA Filter | EMA Filtresi

Exponential Moving Average filters are applied to temperature, humidity, and pressure measurements.

Sıcaklık, nem ve basınç ölçümlerine Üstel Hareketli Ortalama filtresi uygulanmaktadır.

### Kalman Filter | Kalman Filtresi

A Kalman filter is applied to pressure data before the final EMA stage.

Basınç verisine son EMA aşamasından önce Kalman filtresi uygulanmaktadır.

The pressure processing pipeline is approximately:

Basınç işleme zinciri yaklaşık olarak:

```text
Raw Pressure
    ↓
Median Filter
    ↓
Kalman Filter
    ↓
EMA Filter
    ↓
Filtered Pressure
```

---

## Air Density Calculation | Hava Yoğunluğu Hesabı

The firmware calculates air density using filtered temperature, humidity, and atmospheric pressure measurements.

Yazılım, filtrelenmiş sıcaklık, nem ve atmosfer basıncı değerlerini kullanarak hava yoğunluğunu hesaplamaktadır.

Separate gas constants are used for dry air and water vapor:

Kuru hava ve su buharı için ayrı gaz sabitleri kullanılmaktadır:

```text
Rd = 287.058
Rv = 461.495
```

The calculation considers both dry-air partial pressure and water-vapor partial pressure.

Hesaplamada kuru havanın ve su buharının kısmi basınçları ayrı ayrı dikkate alınmaktadır.

---

## GPS | GPS Sistemi

GPS data is parsed using the TinyGPS++ library.

GPS verileri TinyGPS++ kütüphanesi kullanılarak ayrıştırılmaktadır.

The system collects:

- Latitude
- Longitude
- GPS altitude
- Satellite count

Sistem aşağıdaki verileri kullanmaktadır:

- Enlem
- Boylam
- GPS irtifası
- Uydu sayısı

---

## LoRa Telemetry | LoRa Telemetri

Telemetry transmission is implemented using an SX1262-based radio through the RadioLib library.

Telemetri gönderimi RadioLib kütüphanesi üzerinden SX1262 tabanlı LoRa modülü kullanılarak gerçekleştirilmektedir.

The telemetry packet is transmitted approximately every:

Telemetri paketi yaklaşık olarak:

```text
140 ms
```

aralıkla gönderilmektedir.

This corresponds to approximately 6-7 packets per second.

Bu değer saniyede yaklaşık 6-7 telemetri paketine karşılık gelmektedir.

---

## Telemetry Packet Structure | Telemetri Paket Yapısı

The firmware uses a fixed-size binary telemetry packet.

Yazılım sabit boyutlu binary telemetri paketi kullanmaktadır.

```text
Packet Size: 34 bytes
```

The packet includes:

Paket içerisinde:

- Header
- Packet counter
- GPS altitude
- Latitude
- Longitude
- Pressure
- Temperature
- Humidity
- Air density
- Checksum
- Footer

bulunmaktadır.

Numerical values are copied into the telemetry byte buffer using `memcpy`.

Sayısal veriler `memcpy` kullanılarak telemetri byte dizisine aktarılmaktadır.

Example:

```cpp
memcpy(
    byteArr + i,
    &gpsEnlem,
    sizeof(float));
```

A checksum is calculated before the packet footer is added.

Paket sonlandırılmadan önce checksum değeri hesaplanmaktadır.

---

## SD Card Logging | SD Kart Veri Kaydı

Telemetry data is also stored locally on a MicroSD card.

Telemetri verileri aynı zamanda MicroSD karta yerel olarak kaydedilmektedir.

The firmware automatically creates a new CSV file without overwriting previous flight data.

Yazılım, önceki uçuş verilerinin üzerine yazmamak için otomatik olarak yeni bir CSV dosyası oluşturmaktadır.

Example filenames:

```text
VERI000.CSV
VERI001.CSV
VERI002.CSV
...
```

Recorded parameters include:

- Timestamp
- Packet number
- GPS validity
- Latitude
- Longitude
- GPS altitude
- Satellite count
- Pressure
- Temperature
- Humidity
- Air density
- Barometric altitude

Data is periodically flushed to the SD card to reduce excessive write operations.

Aşırı SD kart yazma işlemini azaltmak amacıyla veriler belirli kayıt aralıklarında fiziksel olarak karta yazılmaktadır.

---

## Barometric Altitude | Barometrik İrtifa

The system calculates altitude using filtered atmospheric pressure measurements.

Sistem filtrelenmiş atmosfer basıncı verisini kullanarak barometrik irtifa hesaplamaktadır.

At startup, multiple altitude samples are collected to establish the launch-point reference altitude.

Başlangıçta kalkış noktasının referans irtifasını belirlemek amacıyla birden fazla barometrik irtifa örneği alınmaktadır.

```text
Reference samples: 25
```

Relative altitude is calculated using:

Göreli irtifa:

```text
Relative Altitude =
Current Barometric Altitude - Launch Reference Altitude
```

şeklinde hesaplanmaktadır.

---

## Descent Detection | Düşüş Algılama

The firmware includes descent-detection logic for activating the recovery signaling system.

Firmware, kurtarma sinyal sistemini tetiklemek amacıyla düşüş algılama mantığı içermektedir.

Descent monitoring becomes active after the configured relative-altitude condition is satisfied.

Belirlenen göreli irtifa koşulu sağlandıktan sonra düşüş kontrolü aktif hale gelmektedir.

To avoid false triggering caused by sensor noise, a minimum altitude decrease is required:

Sensör gürültüsünden kaynaklanan yanlış tetiklemeleri azaltmak için minimum irtifa düşüşü:

```text
0.30 m
```

olarak belirlenmiştir.

The system requires multiple consecutive confirmed decreases before activating the recovery indicators.

Kurtarma göstergelerinin aktif edilmesi için birden fazla ardışık düşüş ölçümünün doğrulanması gerekmektedir.

---

## Recovery Signaling | Kurtarma Sinyali

Once descent is confirmed, the firmware activates:

Düşüş doğrulandığında yazılım:

- Siren
- Visual signal beacon

bileşenlerini aktif etmektedir.

The siren remains continuously active.

Siren sürekli aktif durumda tutulmaktadır.

The visual beacon flashes periodically.

Kartalgözü belirli zaman aralıklarında yanıp sönmektedir.

```text
Flash interval: 500 ms
```

---

## Repository Structure | Repo Yapısı

```text
AZAK-Rocket-2026-Payload-Firmware/
│
├── gorevyuku_son.ino
└── README.md
```

---

## Development | Geliştirme

### Elifsena Aycan

All firmware design and implementation in this repository was developed by **Elifsena Aycan**.

Bu repository içerisinde bulunan firmware yazılımının tasarım ve geliştirme sürecinin tamamı **Elifsena Aycan** tarafından gerçekleştirilmiştir.

Responsibilities include:

Katkılar:

- Embedded firmware architecture  
  Gömülü yazılım mimarisi

- Sensor integration  
  Sensör entegrasyonu

- Sensor data acquisition  
  Sensör verilerinin okunması

- Sensor validation  
  Sensör verilerinin doğrulanması

- Median filtering  
  Medyan filtreleme

- EMA filtering  
  EMA filtreleme

- Kalman filtering  
  Kalman filtreleme

- Air-density calculation  
  Hava yoğunluğu hesaplama

- GPS integration  
  GPS entegrasyonu

- LoRa communication  
  LoRa haberleşmesi

- Binary telemetry packet design  
  Binary telemetri paket tasarımı

- `memcpy`-based packet serialization  
  `memcpy` tabanlı paket serileştirme

- Checksum implementation  
  Checksum uygulaması

- SD card logging  
  SD kart veri kaydı

- Automatic CSV file management  
  Otomatik CSV dosya yönetimi

- Barometric altitude calculation  
  Barometrik irtifa hesaplama

- Launch reference altitude calculation  
  Kalkış referans irtifası hesaplama

- Relative altitude tracking  
  Göreli irtifa takibi

- Descent detection logic  
  Düşüş algılama algoritması

- Recovery signaling logic  
  Kurtarma sinyal sistemi

---

## Development Period | Geliştirme Dönemi

**2026**

Developed as part of the AZAK Rocket Team payload and avionics development process.

AZAK Roket Takımı görev yükü ve aviyonik geliştirme süreci kapsamında geliştirilmiştir.

---

## Technologies

`ESP32-S3` `Arduino` `C++` `RadioLib` `SX1262` `LoRa` `TinyGPS++` `GPS` `MS5611` `SHT31` `SPI` `I2C` `UART` `MicroSD` `Kalman Filter` `EMA` `Median Filter` `Telemetry`

---

## Repository Access | Repository Erişimi

This repository is private and contains project-specific embedded firmware.

Bu repository gizlidir ve projeye özel gömülü sistem yazılımı içermektedir.

Source code, communication configuration, telemetry packet structure, recovery logic, and implementation details should not be redistributed without permission.

Kaynak kod, haberleşme konfigürasyonu, telemetri paket yapısı, kurtarma mantığı ve uygulama detayları izin alınmadan yeniden paylaşılmamalıdır.

---

## Team | Takım

**AZAK Rocket Team**
