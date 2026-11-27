/*
* FRDM-K64F + Arduino Multi-function Shield : เกมทรัมเป็ต
* เพลง: Jingle Bells (ท่อนฮุก 26 โน้ต)
* ระบบชนะ: แสดงข้อความ "VICTORY" วิ่งวนอ่านตามปกติพร้อมบรรเลงเพลง Jingle Bells
* ระบบแพ้ : แสดง "LOSE" พร้อมเสียง "ตู๊ด ตูด ตู่ด ตู๊ดดดด~"
* Keil uVision, Arm Compiler 6, C ล้วน, ไฟล์เดียว
*/
#include <stdint.h>
#include "MK64F12.h"

/* ---------- ค่ากำหนดการทำงานของระบบ ---------- */
#define RUN_MODE              0     /* 0 = โหมดเกมหลัก */
#define ENABLE_BUZZER         1     /* 1 = เปิดใช้งานเสียงบัซเซอร์, 0 = ปิดเสียง */
#define BUZZER_ON_LEVEL       0     /* 0 = active-low (ส่งลอจิก 0 แล้วบัซเซอร์ดัง) */
#define BUZZER_VOLUME         14u   /* ความดังพื้นฐานขณะกดปุ่ม (หน่วย % duty cycle, แนะนำ 10-16) */
#define MAX_SAFE_VOL          16u   /* เพดานความดังสูงสุด กันเสียงแหลมดีดตัวใน Active Buzzer */

#define SCROLL_LEFT_TO_RIGHT  0     /* 0 = เลื่อนหน้าต่างข้อความให้เรียงอ่านจากซ้ายไปขวาตามปกติ */
#define SCROLL_SPEED_MS       260u  /* ความเร็วการเลื่อนตัวอักษร (มิลลิวินาทีต่อ 1 ช่อง) */

/* ---------- มาโครเข้าถึงรีจิสเตอร์ฮาร์ดแวร์ของ K64F ---------- */
#define K_REG32(a)   (*(volatile uint32_t *)(a)) /* อ่าน/เขียนรีจิสเตอร์ 32 บิตผ่าน address */
#define K_REG16(a)   (*(volatile uint16_t *)(a)) /* อ่าน/เขียนรีจิสเตอร์ 16 บิตผ่าน address */

#define K_SCGC5      K_REG32(0x40048038u) /* รีจิสเตอร์เปิด Clock ให้พอร์ต GPIO (PORTA-PORTE) */
#define K_SCGC6      K_REG32(0x4004803Cu) /* รีจิสเตอร์เปิด Clock ให้โมดูลต่อพ่วง เช่น FTM0 */
#define K_PORTA      0x40049000u          /* ฐาน address ของ PORTA (ตั้งค่า pin mux / pull-up) */
#define K_PORTB      0x4004A000u          /* ฐาน address ของ PORTB */
#define K_PORTC      0x4004B000u          /* ฐาน address ของ PORTC */
#define K_PORTD      0x4004C000u          /* ฐาน address ของ PORTD */
#define K_GPIOA      0x400FF000u          /* ฐาน address ของ GPIOA (ควบคุมระดับสัญญาณ/ทิศทางขา) */
#define K_GPIOB      0x400FF040u          /* ฐาน address ของ GPIOB */
#define K_GPIOC      0x400FF080u          /* ฐาน address ของ GPIOC */
#define K_GPIOD      0x400FF0C0u          /* ฐาน address ของ GPIOD */

#define K_PCR(p, n)  K_REG32((p) + 4u * (n)) /* รีจิสเตอร์ Pin Control Register ของพอร์ต p ขาที่ n */
#define K_PSOR(g)    K_REG32((g) + 0x04u)    /* Port Set Output Register: เขียน 1 เพื่อเซตขาเป็น 1 */
#define K_PCOR(g)    K_REG32((g) + 0x08u)    /* Port Clear Output Register: เขียน 1 เพื่อเคลียร์ขาเป็น 0 */
#define K_PDIR(g)    K_REG32((g) + 0x10u)    /* Port Data Input Register: อ่านค่าลอจิกปัจจุบันของขา */
#define K_PDDR(g)    K_REG32((g) + 0x14u)    /* Port Data Direction Register: 1 = Output, 0 = Input */

#define K_WDOG_STCTRLH K_REG16(0x40052000u) /* รีจิสเตอร์ควบคุมสถานะ Watchdog Timer */
#define K_WDOG_UNLOCK  K_REG16(0x4005200Eu) /* รีจิสเตอร์ปลดล็อกก่อนแก้ค่า Watchdog Timer */

/* รีจิสเตอร์ของ FlexTimer 0 (FTM0) สำหรับสร้างสัญญาณ PWM บนขา PTA1 (FTM0_CH6, ALT3) */
#define K_FTM0_SC      K_REG32(0x40038000u) /* Status and Control: เลือกแหล่ง clock และ prescaler */
#define K_FTM0_CNT     K_REG32(0x40038004u) /* Counter: ค่าตัวนับปัจจุบันของตัวจับเวลา */
#define K_FTM0_MOD     K_REG32(0x40038008u) /* Modulo: ค่าสูงสุดของตัวนับ กำหนดคาบ/ความถี่ PWM */
#define K_FTM0_C6SC    K_REG32(0x4003803Cu) /* Channel 6 Status and Control: ตั้งโหมด PWM และขั้วสัญญาณ */
#define K_FTM0_C6V     K_REG32(0x40038040u) /* Channel 6 Value: ค่าเปรียบเทียบ กำหนด Duty Cycle */

/* ---------- มาโครควบคุมชิป Shift Register 74HC595 สำหรับ 7-Segment ---------- */
#define LATCH_HI()   (K_PSOR(K_GPIOB) = (1u << 23)) /* ดึงขา LATCH (PTB23) เป็น 1 เพื่อส่งข้อมูลออกหน้าจอ */
#define LATCH_LO()   (K_PCOR(K_GPIOB) = (1u << 23)) /* ลดขา LATCH (PTB23) เป็น 0 ขณะกำลัง shift ข้อมูลเข้า */
#define CLK_HI()     (K_PSOR(K_GPIOC) = (1u << 3))  /* สัญญาณนาฬิกา CLK (PTC3) ขอบขาขึ้น */
#define CLK_LO()     (K_PCOR(K_GPIOC) = (1u << 3))  /* สัญญาณนาฬิกา CLK (PTC3) ขอบขาลง */
#define DATA_HI()    (K_PSOR(K_GPIOC) = (1u << 12)) /* ส่งบิต 1 ทางขา DATA (PTC12) */
#define DATA_LO()    (K_PCOR(K_GPIOC) = (1u << 12)) /* ส่งบิต 0 ทางขา DATA (PTC12) */

#define NOP4()       do { __NOP(); __NOP(); __NOP(); __NOP(); } while (0) /* หน่วงเวลาสั้น ๆ 4 รอบคำสั่ง ให้สัญญาณนิ่ง */

/* ---------- ตารางรหัสลาย 7-Segment (แบบ active-low: บิต 0 = ติด, บิต 1 = ดับ, bit7 = จุดทศนิยม) ---------- */
static const uint8_t SEG_NUM[10] = {
   0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90 /* ลวดลายตัวเลข 0 ถึง 9 */
};
#define SEG_BLANK  0xFF /* ดับทุก segment (ไม่แสดงผล) */
#define SEG_DASH   0xBF /* แสดงขีดกลางตรงกลาง (segment G ติด) */
#define SEG_L      0xC7 /* แสดงตัวอักษร L */
#define SEG_O_UP   0xC0 /* แสดงตัวอักษร O (เหมือนเลข 0) */
#define SEG_S      0x92 /* แสดงตัวอักษร S (เหมือนเลข 5) */
#define SEG_E      0x86 /* แสดงตัวอักษร E */

/* ลวดลายตัวอักษรสำหรับคำว่า "VICTORY" บนจอ 7-Segment */
#define SEG_V      0xC1   /* แสดงรูปทรงคล้ายตัว V */
#define SEG_I      0xF9   /* แสดงตัว I */
#define SEG_C      0xC6   /* แสดงตัว C */
#define SEG_t      0x87   /* แสดงตัว t พิมพ์เล็ก */
#define SEG_r      0xAF   /* แสดงตัว r พิมพ์เล็ก */
#define SEG_y      0x91   /* แสดงตัว y */

/* อาร์เรย์ข้อความเลื่อน "    VICtOry    " (รวมช่องว่างหัว-ท้ายเพื่อให้เลื่อนเข้าและออกจอได้เนียน) */
#define VIC_LEN    15u
static const uint8_t VICTORY_MSG[VIC_LEN] = {
   SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK,
   SEG_V, SEG_I, SEG_C, SEG_t, SEG_O_UP, SEG_r, SEG_y,
   SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK
};

/* ---------- ตัวแปรที่ใช้ร่วมกันระหว่าง Interrupt (SysTick) และลูปหลัก ---------- */
static volatile uint32_t g_ms = 0;                                             /* ตัวนับเวลาแบบมิลลิวินาที (เพิ่มทุก 1 ms) */
static volatile uint8_t  tick_flag = 0;                                        /* ธงบอกว่าเกิด SysTick ครบ 1 ms แล้ว */
static volatile uint8_t  disp[4] = {SEG_BLANK, SEG_BLANK, SEG_BLANK, SEG_BLANK}; /* บัฟเฟอร์ลาย 7-Segment ทั้ง 4 หลัก */
static volatile uint8_t  scan_pos = 0;                                         /* ตำแหน่งหลักที่กำลังสแกน (0..3) */
static uint32_t g_bus_clock = 60000000u;                                       /* ความถี่ Bus Clock (ตั้งต้น 60 MHz แล้วคำนวณจริงใน board_init) */

/* ส่งข้อมูล 8 บิตแบบอนุกรม (MSB -> LSB) เข้าชิป 74HC595 */
static void sr_byte(uint8_t v)
{
   int8_t i;
   for (i = 7; i >= 0; i--) {
       if (v & (1u << i)) { DATA_HI(); } else { DATA_LO(); } /* วางบิตข้อมูลลงขา DATA */
       NOP4();
       CLK_HI();                                              /* สั่งให้ 74HC595 อ่านบิตข้อมูล */
       NOP4();
       CLK_LO();
       NOP4();
   }
}

/* อินเตอร์รัปต์ SysTick ทำงานทุก 1 ms: นับเวลา และทำ Multiplexing สแกน 7-Segment ทีละหลัก */
void SysTick_Handler(void);
void SysTick_Handler(void)
{
   uint8_t p = scan_pos;

   g_ms++;
   tick_flag = 1;

   LATCH_LO();
   sr_byte(disp[p]);          /* ไบต์แรก: ลาย segment ของหลักที่ p */
   sr_byte((uint8_t)(1u << p)); /* ไบต์ที่สอง: เลือกเปิดใช้งานหลักที่ p (0..3) */
   LATCH_HI();

   scan_pos = (uint8_t)((p + 1u) & 3u); /* วนตำแหน่งถัดไป (0 -> 1 -> 2 -> 3 -> 0) */
}

/* ฟังก์ชันหน่วงเวลาแบบรอจนกว่าเวลาจะผ่านไปตาม ms ที่กำหนด (ใช้ g_ms ที่อัปเดตใน SysTick) */
static void delay_ms(uint32_t ms)
{
   uint32_t t = g_ms;
   while ((uint32_t)(g_ms - t) < ms) { }
}

/* ---------- ฟังก์ชันควบคุม LED / ปุ่มกด / บัซเซอร์ ---------- */
static const uint8_t LED_PIN[4] = {1, 3, 2, 0};   /* จับคู่ LED1..LED4 กับขา PTD1, PTD3, PTD2, PTD0 */

/* ตั้งค่าการติด-ดับของ LED 4 ดวงตามบิตมาสก์ mask (แบบ active-low: PCOR = ติด, PSOR = ดับ) */
static void leds_set(uint8_t mask)
{
   uint8_t i;
   for (i = 0; i < 4; i++) {
       if (mask & (1u << i)) { K_PCOR(K_GPIOD) = (1u << LED_PIN[i]); }
       else                  { K_PSOR(K_GPIOD) = (1u << LED_PIN[i]); }
   }
}

/* อ่านสถานะปุ่มกด 3 ปุ่มบนพอร์ต B (PTB3, PTB10, PTB11) แบบ active-low คืนค่าเป็นบิต 1, 2, 4 */
static uint8_t read_buttons(void)
{
   uint32_t d = K_PDIR(K_GPIOB);
   uint8_t m = 0;
   if (!(d & (1u << 3)))  { m |= 1; } /* ปุ่ม 1 (บิต 0 -> ค่า 1) */
   if (!(d & (1u << 10))) { m |= 2; } /* ปุ่ม 2 (บิต 1 -> ค่า 2) */
   if (!(d & (1u << 11))) { m |= 4; } /* ปุ่ม 3 (บิต 2 -> ค่า 4) */
   return m;
}

/* ปิดเสียงบัซเซอร์ และคืนขา PTA1 กลับเป็น GPIO output ในสถานะปิดเสียง */
static void buzzer_off(void)
{
   K_FTM0_SC   = 0u;
   K_FTM0_C6SC = 0u;
   K_PCR(K_PORTA, 1) = 0x100u; /* MUX = 001 (GPIO) */
   if (BUZZER_ON_LEVEL) { K_PCOR(K_GPIOA) = (1u << 1); }
   else                 { K_PSOR(K_GPIOA) = (1u << 1); }
   K_PDDR(K_GPIOA) |= (1u << 1);
}

/* เปิดเสียงบัซเซอร์ด้วย PWM ความถี่ 20 kHz แบบไม่บล็อก (เพื่อใช้คู่กับการเลื่อนข้อความได้) */
static void buzzer_start(uint8_t volume_pct)
{
#if ENABLE_BUZZER
   uint32_t mod_val, duty_val;
   if (volume_pct > MAX_SAFE_VOL) { volume_pct = MAX_SAFE_VOL; } /* จำกัดความดังไม่ให้เกินเพดานปลอดภัย */

   mod_val = (g_bus_clock / 20000u) - 1u;                 /* คำนวณค่า MOD ให้ได้ความถี่ PWM ประมาณ 20 kHz */
   duty_val = (mod_val * (uint32_t)volume_pct) / 100u;    /* คำนวณค่า duty cycle ตามเปอร์เซ็นต์ความดัง */

   K_FTM0_SC   = 0u;
   K_FTM0_CNT  = 0u;
   K_FTM0_MOD  = mod_val;
   K_FTM0_C6SC = (BUZZER_ON_LEVEL == 0) ? 0x24u : 0x28u;  /* Edge-aligned PWM: เลือก low-true หรือ high-true */
   K_FTM0_C6V  = duty_val;

   K_PCR(K_PORTA, 1) = 0x300u;     /* ALT3 = ต่อขา PTA1 เข้ากับสัญญาณ FTM0_CH6 */
   K_FTM0_SC = (1u << 3) | 0u;     /* CLKS = 01 (Bus Clock), PS = 0 (Prescaler = 1) */
#else
   (void)volume_pct;
#endif
}

/* ส่งเสียงบี๊บแบบบล็อกเวลา (เปิดเสียง -> รอ duration_ms -> ปิดเสียง) */
static void play_beep(uint16_t duration_ms, uint8_t volume_pct)
{
   buzzer_start(volume_pct);
   delay_ms(duration_ms);
   buzzer_off();
}

/* กำหนดค่าทั้ง 4 หลักของจอ 7-Segment พร้อมกัน */
static void set_disp4(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
   disp[0] = a; disp[1] = b; disp[2] = c; disp[3] = d;
}

/* ---------- เริ่มต้นระบบบอร์ดและฮาร์ดแวร์ ---------- */
static void board_init(void)
{
   uint32_t outdiv2;

   /* ปลดล็อกและปิด Watchdog Timer เพื่อไม่ให้รีเซ็ตบอร์ดโดยไม่ตั้งใจ */
   __disable_irq();
   K_WDOG_UNLOCK  = 0xC520;
   K_WDOG_UNLOCK  = 0xD928;
   K_WDOG_STCTRLH 
