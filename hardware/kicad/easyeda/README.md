# เปิดแผงใน EasyEDA Standard

เน็ตช้า / เปิด Pro ไม่ได้ — **ใช้ไฟล์ Standard JSON นี้** ไม่ต้องเข้า pro.easyeda.com

อย่าดับเบิลคลิกใน Cursor

## Footprint โมดูล URTK1.2 (K902 breakout)

วัดจากรูปคาลิเปอร์ → `URTK1.2_K902.json`  
วิธีใช้และพินเอาต์: [URTK1.2_README.md](URTK1.2_README.md) · พรีวิว `URTK1.2_preview.svg`  
สร้างใหม่: `python hardware/kicad/gen_urtk12_footprint.py`

## หลัก: EasyEDA มาตรฐาน (เปิดไฟล์นี้)

1. เปิด Chrome ไปที่ **https://easyeda.com/editor** (หรือ https://lceda.cn/editor) แล้วล็อกอิน
2. สร้างหรือเปิดโปรเจกต์
3. เมนู **File → Import → EasyEDA…** (ไฟล์ → นำเข้า → EasyEDA)
4. เลือกไฟล์นี้เท่านั้น (ไฟล์ `.json` ไม่ใช่ `.epro`)  
   `c:\project\ESP32_Rover_NEZ\hardware\kicad\easyeda\gnss-head.json`  
   หรือแตก `gnss-head-easyeda-std.zip` แล้วชี้ `gnss-head.json`
5. เปิดบอร์ดแล้วกด Fit — ชั้นล่าง (น้ำเงิน) เปิดไว้ถ้ายังไม่เห็นลายหลัง

เปิดแล้วควรเห็นวงกลม 120 มม. ลายทองแดง และชิ้นส่วน:

- M1 / กรอบ K902 38×38 กลางแผง แถวพินห่าง 34 มม.
- ซ็อกเก็ตตัวเมีย 1×9 + 1×10 และ ESP32 2×15
- J7 JST-PH 6P ตัวผู้สำรองซ้ายของ K902
- HC-12 ใต้ K902, USB-C 5V บนซ้าย (แพด +/−/K)
- J5 แบตริมซ้าย
- **J8 SW3 PWR** ริมซ้ายเหนือ J5 — แจ็คสวิตช์กดนอก (K–GND) ไม่ใช่แบต ไม่มี 6×6 บนแผง

ลาย **แดง = หน้า** UART2 + 5V ขึ้น ESP  
ลาย **น้ำเงิน = หลัง** 5V / UART1 ไป HC-12 / PSU_K ไป J8

ไฟล์นี้ **ไม่มี COPPERAREA** เททองแดง — ลายเส้นโชว์ชัด ถ้าจะเท GND ให้เทในตัว editor เองทีหลัง

## สำรอง: EasyEDA Pro (ไม่จำเป็น)

ถ้าเน็ตดีและอยากใช้ Pro ทีหลัง: `gnss-head.epro` หรือ `gnss-head-easyeda-pro.zip` ที่ https://pro.easyeda.com/ → Import → EasyEDA Pro
