#include <MS5611.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <RadioLib.h>
#include <SPI.h>
#include <SD.h>
#include <cstring>
#include <math.h>

// ======================================================
// PIN TANIMLARI
// ======================================================

#define SDA_PIN 14
#define SCL_PIN 15

#define GPS_RX_PIN 18
#define GPS_TX_PIN 17

#define LORA_NSS 10
#define LORA_MOSI 11
#define LORA_SCK 12
#define LORA_MISO 13
#define LORA_RST 8
#define LORA_BUSY 9
#define LORA_DIO1 7

#define SD_CS_PIN 40

#define RXEN_PIN 47
#define TXEN_PIN 21

#define BUZZER_PIN 41
#define SIGNAL_LED 39
#define SIREN 38

// 140 ms = saniyede yaklaşık 6-7 paket
#define TX_INTERVAL_MS 140

// Her 5 kayıtta bir dosya SD karta fiziksel olarak yazılır.
#define SD_FLUSH_KAYIT_SAYISI 5

#define MEDIAN_SIZE 5

// Paket:
// 1 header + 2 paket sayacı + 7 float + 1 checksum + 2 footer
#define TELEMETRI_PAKET_BOYUTU 34

// ======================================================
// SENSÖR VE HABERLEŞME NESNELERİ
// ======================================================

Adafruit_SHT31 sht31 = Adafruit_SHT31();

// Arduino-MS5611-dev kütüphanesi
MS5611 BasincSensor;

HardwareSerial GPS_Serial(1);
TinyGPSPlus gps;

SPIClass loraSPI(FSPI);

SX1262 radio = new Module(
  LORA_NSS,
  LORA_DIO1,
  LORA_RST,
  LORA_BUSY,
  loraSPI);

// ======================================================
// SD KART NESNELERİ VE DURUMU
// ======================================================

File veriDosyasi;
bool sdKartAktif = false;
uint8_t sdFlushSayaci = 0;
char sdDosyaAdi[20] = "/VERI000.CSV";

// ======================================================
// DURUM DEĞİŞKENLERİ
// ======================================================

unsigned long sonGondermeZamani = 0;
uint16_t paketSayaci = 0;

bool sht31Aktif = false;
bool ms5611Aktif = false;

// ======================================================
// GPS DEĞİŞKENLERİ
// ======================================================

float gpsEnlem = 0.0f;
float gpsBoylam = 0.0f;
float gpsIrtifa = 0.0f;

// ======================================================
// HAM VE FİLTRELENMİŞ SENSÖR VERİLERİ
// ======================================================

float rawTempC = 25.0f;
float rawHumidity = 50.0f;
float rawPressurePa = 101325.0f;

float filteredTempC = 25.0f;
float filteredHumidity = 50.0f;
float filteredPressurePa = 101325.0f;

float density = 0.0f;

// MS5611'in kendi sıcaklık değeri
float ms5611Sicaklik = 0.0f;

// Barometrik irtifa
float barometrikIrtifa = 0.0f;
float filtreliBarometrikIrtifa = 0.0f;
float oncekiBarometrikIrtifa = 0.0f;
float kalkisBarometrikIrtifa = 0.0f;
float goreliBarometrikIrtifa = 0.0f;

bool kalkisIrtifasiBelirlendi = false;
uint8_t kalkisIrtifaOrnekSayisi = 0;
float kalkisIrtifaToplami = 0.0f;

const uint8_t KALKIS_REFERANS_ORNEK_SAYISI = 25;

bool kartalgozuSirenKilitli = false;
bool dususKontroluAktif = false;

uint8_t ardArdaDususSayaci = 0;
bool kartalgozuDurumu = false;

unsigned long sonKartalgozuDegisimZamani = 0;

const unsigned long KARTALGOZU_FLASH_INTERVAL_MS = 500;

// Bir ölçümün gerçek düşüş sayılması için minimum fark
const float MIN_DUSUS_FARKI = 0.30f;

// Son geçerli ham basınç
long ms5611BasincPa = 101325;

// ======================================================
// FİLTRE BUFFERLARI
// ======================================================

float pressureMedianBuffer[MEDIAN_SIZE] = {
  101325.0f,
  101325.0f,
  101325.0f,
  101325.0f,
  101325.0f
};

float humidityMedianBuffer[MEDIAN_SIZE] = {
  50.0f,
  50.0f,
  50.0f,
  50.0f,
  50.0f
};

// Kuru hava ve su buharı gaz sabitleri
const float Rd = 287.058f;
const float Rv = 461.495f;

// ======================================================
// EMA FİLTRESİ
// ======================================================

class EMAFilter {
private:
  float alpha;
  float filteredValue;
  bool initialized;

public:
  explicit EMAFilter(float a)
    : alpha(a),
      filteredValue(0.0f),
      initialized(false) {
  }

  float update(float input) {
    if (!initialized) {
      filteredValue = input;
      initialized = true;
    } else {
      filteredValue =
        alpha * input + (1.0f - alpha) * filteredValue;
    }

    return filteredValue;
  }
};

// ======================================================
// KALMAN FİLTRESİ
// ======================================================

class KalmanFilter {
private:
  float Q;
  float R;
  float X;
  float P;
  float K;
  bool initialized;

public:
  KalmanFilter(float q, float r)
    : Q(q),
      R(r),
      X(0.0f),
      P(1.0f),
      K(0.0f),
      initialized(false) {
  }

  float update(float measurement) {
    if (!initialized) {
      X = measurement;
      initialized = true;
    }

    P = P + Q;
    K = P / (P + R);

    X = X + K * (measurement - X);
    P = (1.0f - K) * P;

    return X;
  }
};

EMAFilter tempEMA(0.15f);
EMAFilter humidityEMA(0.20f);
EMAFilter pressureEMA(0.20f);

KalmanFilter pressureKalman(0.01f, 0.8f);

// ======================================================
// MEDYAN FİLTRESİ
// ======================================================

float medianFilter(
  float newValue,
  float buffer[],
  int &bufferIndex) {
  buffer[bufferIndex] = newValue;
  bufferIndex++;

  if (bufferIndex >= MEDIAN_SIZE) {
    bufferIndex = 0;
  }

  float temp[MEDIAN_SIZE];

  for (int i = 0; i < MEDIAN_SIZE; i++) {
    temp[i] = buffer[i];
  }

  for (int i = 0; i < MEDIAN_SIZE - 1; i++) {
    for (int j = i + 1; j < MEDIAN_SIZE; j++) {
      if (temp[j] < temp[i]) {
        float gecici = temp[i];
        temp[i] = temp[j];
        temp[j] = gecici;
      }
    }
  }

  return temp[MEDIAN_SIZE / 2];
}

// Her buffer için ayrı indeks
int pressureBufferIndex = 0;
int humidityBufferIndex = 0;

// ======================================================
// CHECKSUM
// ======================================================

uint8_t checksumHesapla(
  const byte *data,
  int length) {
  uint16_t toplam = 0;

  for (int i = 0; i < length; i++) {
    toplam += data[i];
  }

  return static_cast<uint8_t>(toplam & 0xFF);
}

// ======================================================
// GPS OKUMA
// ======================================================

void GPS_Verilerini_Oku() {
  while (GPS_Serial.available() > 0) {
    char karakter = GPS_Serial.read();
    gps.encode(karakter);
  }

  if (gps.location.isValid()) {
    gpsEnlem = gps.location.lat();
    gpsBoylam = gps.location.lng();
  }

  if (gps.altitude.isValid()) {
    gpsIrtifa = gps.altitude.meters();
  }
}

// ======================================================
// MS5611 OKUMA
// Arduino-MS5611-dev API
// ======================================================

bool MS5611_Verilerini_Oku() {
  if (!ms5611Aktif) {
    return false;
  }

  long yeniBasincPa = BasincSensor.readPressure(true);
  float yeniSicaklik = BasincSensor.readTemperature(true);

  bool basincGecerli =
    yeniBasincPa > 10000 && yeniBasincPa < 120000;

  bool sicaklikGecerli =
    !isnan(yeniSicaklik) && yeniSicaklik > -50.0f && yeniSicaklik < 100.0f;

  if (!basincGecerli) {
    Serial.print("MS5611 gecersiz ham basinc: ");
    Serial.println(yeniBasincPa);
    return false;
  }

  if (!sicaklikGecerli) {
    Serial.print("MS5611 gecersiz sicaklik: ");
    Serial.println(yeniSicaklik);
    return false;
  }

  ms5611BasincPa = yeniBasincPa;
  ms5611Sicaklik = yeniSicaklik;

  rawPressurePa = static_cast<float>(ms5611BasincPa);

  barometrikIrtifa =
    BasincSensor.getAltitude(ms5611BasincPa);

  return true;
}

// ======================================================
// TÜM SENSÖRLERİ OKU VE FİLTRELE
// ======================================================

bool Sensor_Verilerini_Oku() {
  // ---------------- SHT31 ----------------

  if (sht31Aktif) {
    float yeniSicaklik = sht31.readTemperature();
    float yeniNem = sht31.readHumidity();

    if (!isnan(yeniSicaklik)) {
      rawTempC = yeniSicaklik;
    } else {
      Serial.println("SHT31 sicaklik okuma hatasi!");
    }

    if (!isnan(yeniNem)) {
      rawHumidity = yeniNem;
    } else {
      Serial.println("SHT31 nem okuma hatasi!");
    }
  }

  // ---------------- MS5611 ----------------

  if (!MS5611_Verilerini_Oku()) {
    Serial.println(
      "MS5611 okuma hatasi, son gecerli basinc kullaniliyor.");
  }

  // ---------------- FİLTRELER ----------------

  filteredTempC = tempEMA.update(rawTempC);

  float humidityMedian = medianFilter(
    rawHumidity,
    humidityMedianBuffer,
    humidityBufferIndex);

  filteredHumidity =
    humidityEMA.update(humidityMedian);

  float pressureMedian = medianFilter(
    rawPressurePa,
    pressureMedianBuffer,
    pressureBufferIndex);

  float pressureKalmanValue =
    pressureKalman.update(pressureMedian);

  filteredPressurePa =
    pressureEMA.update(pressureKalmanValue);
    filtreliBarometrikIrtifa =
    BasincSensor.getAltitude(
        static_cast<long>(filteredPressurePa));

  // ---------------- HAVA YOĞUNLUĞU ----------------

  float sicaklikKelvin =
    filteredTempC + 273.15f;

  float doymusBuharBasinciHpa =
    6.112f * exp((17.67f * filteredTempC) / (filteredTempC + 243.5f));

  float doymusBuharBasinciPa =
    doymusBuharBasinciHpa * 100.0f;

  float suBuhariKismiBasinci =
    (filteredHumidity / 100.0f) * doymusBuharBasinciPa;

  float kuruHavaKismiBasinci =
    filteredPressurePa - suBuhariKismiBasinci;

  density =
    (kuruHavaKismiBasinci / (Rd * sicaklikKelvin)) + (suBuhariKismiBasinci / (Rv * sicaklikKelvin));

  return true;
}

// ======================================================
// SD KART BAŞLATMA VE CSV KAYDI
// ======================================================

bool SD_Karti_Baslat() {
  // LoRa ve SD aynı SPI veri yolunu paylaşır; CS pinleri ayrıdır.
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);

  if (!SD.begin(SD_CS_PIN, loraSPI, 10000000)) {
    sdKartAktif = false;
    Serial.println("SD kart baslatma hatasi! LoRa calismaya devam edecek.");
    return false;
  }

  // Her açılışta önceki verilerin üzerine yazmamak için
  // kullanılmayan ilk VERIxxx.CSV dosya adı seçilir.
  bool uygunDosyaBulundu = false;

  for (uint16_t dosyaNo = 0; dosyaNo < 1000; dosyaNo++) {
    snprintf(
      sdDosyaAdi,
      sizeof(sdDosyaAdi),
      "/VERI%03u.CSV",
      dosyaNo);

    if (!SD.exists(sdDosyaAdi)) {
      uygunDosyaBulundu = true;
      break;
    }
  }

  if (!uygunDosyaBulundu) {
    sdKartAktif = false;
    Serial.println("SD kartta yeni dosya adi olusturulamadi!");
    return false;
  }

  veriDosyasi = SD.open(sdDosyaAdi, FILE_WRITE);

  if (!veriDosyasi) {
    sdKartAktif = false;
    Serial.println("SD kart veri dosyasi acilamadi!");
    return false;
  }

  // CSV sütun başlıkları
  veriDosyasi.println(
    "zaman_ms,paket_no,gps_gecerli,enlem,boylam,gps_irtifa_m,uydu_sayisi,"
    "basinc_hpa,sht_sicaklik_c,ms5611_sicaklik_c,nem_yuzde,"
    "hava_yogunlugu_kg_m3,barometrik_irtifa_m");

  veriDosyasi.flush();
  sdKartAktif = true;
  sdFlushSayaci = 0;

  Serial.print("SD kart hazir. Veri dosyasi: ");
  Serial.println(sdDosyaAdi);

  return true;
}

void SD_Verilerini_Kaydet() {
  if (!sdKartAktif) {
    return;
  }

  // Dosya herhangi bir nedenle kapanmışsa yeniden açmayı dene.
  if (!veriDosyasi) {
    veriDosyasi = SD.open(sdDosyaAdi, FILE_APPEND);

    if (!veriDosyasi) {
      sdKartAktif = false;
      Serial.println("SD dosyasi yeniden acilamadi. Kayit durduruldu!");
      return;
    }
  }

  veriDosyasi.print(millis());
  veriDosyasi.print(',');

  veriDosyasi.print(paketSayaci);
  veriDosyasi.print(',');

  veriDosyasi.print(gps.location.isValid() ? 1 : 0);
  veriDosyasi.print(',');

  veriDosyasi.print(gpsEnlem, 6);
  veriDosyasi.print(',');

  veriDosyasi.print(gpsBoylam, 6);
  veriDosyasi.print(',');

  veriDosyasi.print(gpsIrtifa, 2);
  veriDosyasi.print(',');

  veriDosyasi.print(gps.satellites.value());
  veriDosyasi.print(',');

  veriDosyasi.print(filteredPressurePa / 100.0f, 2);
  veriDosyasi.print(',');

  veriDosyasi.print(filteredTempC, 2);
  veriDosyasi.print(',');

  veriDosyasi.print(ms5611Sicaklik, 2);
  veriDosyasi.print(',');

  veriDosyasi.print(filteredHumidity, 2);
  veriDosyasi.print(',');

  veriDosyasi.print(density, 4);
  veriDosyasi.print(',');

  veriDosyasi.println(barometrikIrtifa, 2);

  sdFlushSayaci++;

  if (sdFlushSayaci >= SD_FLUSH_KAYIT_SAYISI) {
    veriDosyasi.flush();
    sdFlushSayaci = 0;
  }
}

// ======================================================
// LORA TELEMETRİ PAKETİ
// ======================================================

void Lora_Paketi_Gonder() {
  radio.standby();
  delay(5);

  byte byteArr[TELEMETRI_PAKET_BOYUTU];
  int i = 0;

  // Header
  byteArr[i++] = 0xAB;

  // Paket sayacı
  memcpy(
    byteArr + i,
    &paketSayaci,
    sizeof(uint16_t));
  i += sizeof(uint16_t);

  // GPS
  memcpy(
    byteArr + i,
    &gpsIrtifa,
    sizeof(float));
  i += sizeof(float);

  memcpy(
    byteArr + i,
    &gpsEnlem,
    sizeof(float));
  i += sizeof(float);

  memcpy(
    byteArr + i,
    &gpsBoylam,
    sizeof(float));
  i += sizeof(float);

  // Sensör verileri
  float basincHpa =
    filteredPressurePa / 100.0f;

  memcpy(
    byteArr + i,
    &basincHpa,
    sizeof(float));
  i += sizeof(float);

  memcpy(
    byteArr + i,
    &filteredTempC,
    sizeof(float));
  i += sizeof(float);

  memcpy(
    byteArr + i,
    &filteredHumidity,
    sizeof(float));
  i += sizeof(float);

  memcpy(
    byteArr + i,
    &density,
    sizeof(float));
  i += sizeof(float);

  // Checksum
  byte checksum = checksumHesapla(byteArr, i);
byteArr[i++] = checksum;

  // Footer
  byteArr[i++] = 0x0D;
  byteArr[i++] = 0x0A;

  if (i != TELEMETRI_PAKET_BOYUTU) {
    Serial.print("Paket boyutu hatasi! Hesaplanan: ");
    Serial.println(i);
    return;
  }

  int state =
    radio.transmit(byteArr, i);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.print("LoRa gonderildi. Paket No: ");
    Serial.println(paketSayaci);
  } else {
    Serial.print("LoRa TX Error: ");
    Serial.println(state);
  }
}

// ======================================================
// SETUP
// ======================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---------------- LED ----------------

  pinMode(SIGNAL_LED, OUTPUT);
  digitalWrite(SIGNAL_LED, LOW);
  Serial.println("0 - KARTALGOZU LOW");

  // ---------------- SIREN ----------------

  pinMode(SIREN, OUTPUT);
  digitalWrite(SIREN, LOW);
  Serial.println("0 -SİREN LOW");

  Serial.println("1 - VERICI kod basladi");

  // ---------------- BUZZER ----------------

  pinMode(BUZZER_PIN, OUTPUT);

  tone(BUZZER_PIN, 2700);
  delay(5000);
  noTone(BUZZER_PIN);

  // ---------------- I2C ----------------

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  Serial.println("2 - I2C baslatildi");

  // ---------------- GPS ----------------

  GPS_Serial.begin(
    9600,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN);

  Serial.println("3 - GPS UART baslatildi");

  // ---------------- SHT31 ----------------

  if (sht31.begin(0x44)) {
    sht31Aktif = true;
    Serial.println(
      "4 - SHT31 bulundu. Adres: 0x44");
  } else if (sht31.begin(0x45)) {
    sht31Aktif = true;
    Serial.println(
      "4 - SHT31 bulundu. Adres: 0x45");
  } else {
    sht31Aktif = false;
    Serial.println("4 - SHT31 bulunamadi.");
  }

  // ---------------- MS5611 ----------------

  Serial.println("5 - MS5611 baslatiliyor");

  BasincSensor.begin(MS5611_ULTRA_HIGH_RES);
  delay(100);

  long ilkBasincPa = 0;

  // İlk dönüşümleri at
  for (int i = 0; i < 10; i++) {
    ilkBasincPa =
      BasincSensor.readPressure(true);

    delay(50);
  }

  if (
    ilkBasincPa > 10000 && ilkBasincPa < 120000) {
    ms5611Aktif = true;
    ms5611BasincPa = ilkBasincPa;
    rawPressurePa =
      static_cast<float>(ilkBasincPa);

    Serial.print(
      "5 - MS5611 bulundu. Baslangic basinci: ");
    Serial.print(
      ilkBasincPa / 100.0f,
      2);
    Serial.println(" hPa");
  } else {
    ms5611Aktif = false;

    Serial.print(
      "5 - MS5611 baslatma hatasi! Ham basinc: ");
    Serial.println(ilkBasincPa);
  }

  // ---------------- LORA SPI ----------------

  loraSPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_NSS);

  pinMode(LORA_NSS, OUTPUT);
  digitalWrite(LORA_NSS, HIGH);

  // ---------------- SD KART ----------------
pinMode(SD_CS_PIN, OUTPUT);
digitalWrite(SD_CS_PIN, HIGH);
  
  // Manuel LoRa reset
  pinMode(LORA_RST, OUTPUT);

  digitalWrite(LORA_RST, HIGH);
  delay(100);

  digitalWrite(LORA_RST, LOW);
  delay(10);

  digitalWrite(LORA_RST, HIGH);
  delay(100);

  Serial.println(
    "6 - SPI ve LoRa reset tamam");

  int state = radio.begin(
    870.0,
    125.0,
    7,
    5,
    0x18,
    22);

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(
      "7 - LoRa baslatildi. Frekans: 870 MHz");
      state = radio.setCurrentLimit(140);

if (state != RADIOLIB_ERR_NONE) {
  Serial.print("Current limit hatasi: ");
  Serial.println(state);
}

    radio.setRfSwitchPins(
      RXEN_PIN,
      TXEN_PIN);

    radio.standby();
    delay(100);
  } else {
    Serial.print(
      "LoRa baslatma hatasi: ");
    Serial.println(state);

    while (true) {
      delay(10);
    }
  }
SD_Karti_Baslat();
  Serial.println(
    "8 - VERICI sistem baslatildi");
}

// ======================================================
// LOOP
// ======================================================

void loop() {
  GPS_Verilerini_Oku();
  // ======================================================
// KARTALGOZU FLASH KONTROLU
// ======================================================

if (kartalgozuSirenKilitli) {

    // Siren sürekli açık kalsın
    digitalWrite(SIREN, HIGH);

    unsigned long kartalgozuSimdikiZaman = millis();

    if (kartalgozuSimdikiZaman -
            sonKartalgozuDegisimZamani >=
        KARTALGOZU_FLASH_INTERVAL_MS) {

        sonKartalgozuDegisimZamani =
            kartalgozuSimdikiZaman;

        kartalgozuDurumu =
            !kartalgozuDurumu;

        digitalWrite(
            SIGNAL_LED,
            kartalgozuDurumu ? HIGH : LOW);
    }
}

  unsigned long simdikiZaman = millis();

  if (
    simdikiZaman - sonGondermeZamani >= TX_INTERVAL_MS) {
    sonGondermeZamani = simdikiZaman;

    Sensor_Verilerini_Oku();
    // Kalkış noktasının barometrik irtifasını belirle
if (ms5611Aktif && !kalkisIrtifasiBelirlendi) {

    kalkisIrtifaToplami += filtreliBarometrikIrtifa;
    kalkisIrtifaOrnekSayisi++;

    if (kalkisIrtifaOrnekSayisi >= KALKIS_REFERANS_ORNEK_SAYISI) {

        kalkisBarometrikIrtifa =
            kalkisIrtifaToplami /
            KALKIS_REFERANS_ORNEK_SAYISI;

        kalkisIrtifasiBelirlendi = true;

        Serial.print("Kalkis referans irtifasi: ");
        Serial.print(kalkisBarometrikIrtifa, 2);
        Serial.println(" m");
    }
}
if (kalkisIrtifasiBelirlendi) {

    goreliBarometrikIrtifa =
        filtreliBarometrikIrtifa -
        kalkisBarometrikIrtifa;
}
if (!dususKontroluAktif &&
    kalkisIrtifasiBelirlendi &&
    goreliBarometrikIrtifa >= 2400.0f) {

    dususKontroluAktif = true;

    ardArdaDususSayaci = 0;

    oncekiBarometrikIrtifa =
        filtreliBarometrikIrtifa;

    Serial.println(
        "2400 M GECILDI - DUSUS KONTROLU AKTIF");
}
    if (ms5611Aktif &&
    dususKontroluAktif &&
    !kartalgozuSirenKilitli) {

    float irtifaFarki =
        filtreliBarometrikIrtifa -
        oncekiBarometrikIrtifa;

    // En az 30 cm düşüş varsa gerçek düşüş kabul et
    if (irtifaFarki <= -MIN_DUSUS_FARKI) {

        ardArdaDususSayaci++;

        Serial.print(
            "Ard arda dusus sayisi: ");

        Serial.println(
            ardArdaDususSayaci);
    }

    else {

        // Yükselme, sabit kalma veya çok küçük oynama
        // görülürse sayaç sıfırlanır.
        ardArdaDususSayaci = 0;
    }

    // 3 gerçek düşüş arka arkaya görülürse tetikle
    if (ardArdaDususSayaci >= 3) {

        kartalgozuSirenKilitli = true;

        // Siren sürekli açık
        digitalWrite(SIREN, HIGH);

        // Kartalgözü ilk anda yanık başlasın
        kartalgozuDurumu = true;
        digitalWrite(SIGNAL_LED, HIGH);

        // Flaş zamanlayıcısını başlat
        sonKartalgozuDegisimZamani = millis();

        Serial.println(
            "DUSUS ONAYLANDI - "
            "KARTALGOZU FLASH VE SIREN AKTIF");
    }

    // Bir sonraki karşılaştırma için mevcut irtifayı sakla
    oncekiBarometrikIrtifa =
        filtreliBarometrikIrtifa;
}

    Lora_Paketi_Gonder();
    SD_Verilerini_Kaydet();

    Serial.print("GPS: ");
    Serial.print(gpsEnlem, 6);
    Serial.print(", ");
    Serial.print(gpsBoylam, 6);

    Serial.print(" | GPS Irtifa: ");
    Serial.print(gpsIrtifa, 2);
    Serial.print(" m");

    Serial.print(" | Uydu: ");
    Serial.print(gps.satellites.value());

    Serial.print(" | Basinc: ");
    Serial.print(
      filteredPressurePa / 100.0f,
      2);
    Serial.print(" hPa");

    Serial.print(" | SHT Sicaklik: ");
    Serial.print(filteredTempC, 2);
    Serial.print(" C");

    Serial.print(" | MS5611 Sicaklik: ");
    Serial.print(ms5611Sicaklik, 2);
    Serial.print(" C");

    Serial.print(" | Nem: ");
    Serial.print(filteredHumidity, 2);
    Serial.print(" %");

    Serial.print(" | Yogunluk: ");
    Serial.print(density, 4);
    Serial.print(" kg/m3");

    Serial.print(" | Baro Irtifa: ");
    Serial.print(barometrikIrtifa, 2);
    Serial.println(" m");

    paketSayaci++;
  }
}