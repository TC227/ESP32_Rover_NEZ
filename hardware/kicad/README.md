# เปิดวงจรนี้ใน KiCad

1. ติดตั้ง **KiCad 8 หรือ 9** (แนะนำ 9 — ไฟล์เป็น S-expression รุ่น 9)
2. **File → Open**
3. เลือก  
   `c:\project\ESP32_Rover_NEZ\hardware\kicad\gnss-head.kicad_pro`
4. เปิด **PCB Editor** — แผงพาหะ `gnss-head.kicad_pcb`
   - วงกลม **DIA 120 mm** ศูนย์กลาง `(60, 60)` — **K902 อยู่กลางเป๊ะ**
   - ด้านหน้า: K902 48×42 + ซ็อกเก็ตตัวเมีย 1×9+1×10 + **J7 JST-PH 6P ตัวผู้สำรอง** + ESP32 ซ็อกเก็ตตัวเมีย 2×15
   - ด้านหลัง: USB-C 5V บนซ้าย (USB +Y แพด +/−/K) + HC-12 ใต้ K902 (เสา +X) + **BAT J5 PH2.0** + **J8 SW3 PWR PH2.0** (สวิตช์ภายนอก ไม่บัดกรี 6×6 บนแผง)
   - เปิดใน EasyEDA **Standard**: `easyeda/gnss-head.json` — File → Import → EasyEDA (ดู `easyeda/README.md`) ไม่ต้องใช้ Pro
5. สลับดูด้านหลัง: กด **B** หรือ View → Flip Board
6. **Edit → Fill All Zones** ครั้งหนึ่งหลังเปิด
7. อย่ากด Update PCB from Schematic ทับเลย์เอาต์นี้ — schematic ชุด SMT WROOM คนละสไตล์

สร้าง PCB ใหม่จากสคริปต์:

`python c:\project\ESP32_Rover_NEZ\hardware\kicad\gen_carrier_pcb.py`

GPIO16/17 = K902 COM2 TTL · GPIO26/27/25 = HC-12 · J5 แบตเป็น **PH2.0 ไม่ใช่ XH**

ลายปริ้น (ทองแดง) ส่งออกแล้วที่  
`hardware/kicad/plot/gnss-head-F_Cu.svg` (ด้านหน้า)  
`hardware/kicad/plot/gnss-head-B_Cu.svg` (ด้านหลัง พลิกกระจก)

สร้างใหม่: `python hardware/kicad/plot_copper_svg.py`  
ใน KiCad: **File → Plot** เลือก Gerber/SVG ชั้น F.Cu B.Cu Edge.Cuts แล้ว **Fill All Zones** ก่อนส่งโรงงาน
