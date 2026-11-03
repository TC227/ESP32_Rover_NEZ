# Hardware — GNSS radio PCB (BASE + ROVER)

แผงควบคุม ESP32 + HC-12 สำหรับคู่ `ESP32_Rover_NEZ`  
ฮาร์ดแวร์ใบเดียว แฟลชคนละ env — **ไม่มีสวิตช์วิทยุบน PCB**

| ไฟล์ | ใช้ทำอะไร |
| --- | --- |
| [schematic.md](schematic.md) | แผนวงจร, พาวเวอร์, ตารางขา, เสาอากาศ, คำเตือน, ซิลค์คอนเนกเตอร์ |
| [pinmap.csv](pinmap.csv) | ตารางเน็ตโมดูล ↔ GPIO |
| [bom.csv](bom.csv) | BOM ชิ้นต่อวงจร (คอนเนกเตอร์ LDO แคป USB) |
| [gnss-radio.net](gnss-radio.net) | KiCad sexp netlist (เชื่อชื่อเน็ต; เบอร์ขา CP2102N ทวน datasheet) |

GPIO จากเฟิร์มแวร์เท่านั้น: **16/17 = K902 COM2 TTL**, **26/27/25 = HC-12**, BLE ในชิป ESP32
