# URTK1.2 / SinoGNSS K902 — EasyEDA footprint

ไฟล์: `URTK1.2_K902.json` (EasyEDA **Standard** package)

## วิธีเปิด

1. ไป https://easyeda.com/editor แล้วล็อกอิน
2. **File → Import → EasyEDA…**
3. เลือก `hardware/kicad/easyeda/URTK1.2_K902.json`
4. บันทึกลง Library แล้ววางบนบอร์ดได้

## ขนาดจากรูปคาลิเปอร์ (ประมาณ)

| รายการ | ค่า |
| --- | --- |
| ตัวบอร์ด (ไม่รวม SMA/USB) | **38.0 × 38.0 mm** |
| รูยึด (ศูนย์ถึงศูนย์) | **30.0 × 30.0 mm** Ø2.2 |
| พิตช์หัวพิน | **2.54 mm** |
| ระยะแถวซ้าย–ขวา (ศูนย์แพด) | **34.0 mm** |
| ทิศ SMA / RF | **+Y** |
| ทิศ USB-C + JST | **−Y** |

## พินเอาต์

### ซ้าย (ขา 1–9) — SPI / UART2  ปลาย +Y = SMA

| # | ชื่อ |
| --- | --- |
| 1 | SCK |
| 2 | TX_MISO |
| 3 | RX_MOSI |
| 4 | CS |
| 5 | RX2 |
| 6 | TX2 |
| 7 | 3V3 |
| 8 | 5V |
| 9 | GND |

### ขวา (ขา 10–19) — control / I2C  ปลาย +Y = SMA

| # | ชื่อ |
| --- | --- |
| 10 | INT |
| 11 | SAFE |
| 12 | RST |
| 13 | PPS |
| 14 | RTK |
| 15 | FENCE |
| 16 | SCL |
| 17 | SDA |
| 18 | 3V3 |
| 19 | GND |

## หมายเหตุ

- ค่าจากรูปคาลิเปอร์ (±0.5 mm) — ถ้าจะสั่งโรงงานให้วัดซ้ำด้วยดิจิทัลคาลิเปอร์
- กรอบ `gnss-head` เดิมใช้ K902 **48×42** / แถวพิน **45 mm** คนละชุดกับ footprint นี้
- TX2/RX2 = COM2 TTL 3.3 V (ต่อ ESP32 UART2)
- JST บนบอร์ด: 5V / TX1 / RX1 / NC / NC / GND

สร้างใหม่: `python hardware/kicad/gen_urtk12_footprint.py`
