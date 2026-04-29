#include <REG51.H>
#include <intrins.h>

/*
  STC15W204S CW keyer
  Keil C51

  Physical pin mapping:
    P5.5 -> paddle input 1, active LOW
    P5.4 -> paddle input 2, active LOW

    P3.3 -> KEY output + onboard LED
    P3.2 -> sidetone output to passive piezo / transistor speaker driver

    P3.0 -> DOWN button, active LOW
    P3.1 -> UP button, active LOW

  Paddle layouts:

    RIGHT layout:
      P5.5 -> DASH
      P5.4 -> DOT

    LEFT layout:
      P5.5 -> DOT
      P5.4 -> DASH

  Normal mode:
    Short DOWN -> automatic message: CQ CQ DE EW6ML EW6ML EW6ML K
    Short UP   -> automatic message: TNX QSO 73

    Long DOWN  -> speed DOWN
    Long UP    -> speed UP

  Tone adjustment:
    Hold physical P5.5 and press:
      P3.0 -> tone DOWN
      P3.1 -> tone UP

  Paddle layout switching:
    Hold physical P5.4 and press:
      P3.0 -> LEFT layout
      P3.1 -> RIGHT layout

    Confirmation: 1 second sidetone signal.
*/

typedef unsigned char  u8;
typedef unsigned int   u16;
typedef unsigned long  u32;
typedef signed char    s8;

/*
  Set this to the clock frequency selected in STC-ISP.
*/
#define FOSC_HZ             11059200UL

#define DEFAULT_WPM         20
#define MIN_WPM             5
#define MAX_WPM             35

#define DEFAULT_TONE_HZ     1000
#define MIN_TONE_HZ         0
#define MAX_TONE_HZ         2500
#define TONE_STEP_HZ        100

#define DEBOUNCE_MS         20
#define LONG_PRESS_MS       1000

/*
  Timer0 interrupt period: 100 us.
*/
#define TIMER0_TICK_US      100UL
#define TIMER0_TICKS_PER_SECOND (1000000UL / TIMER0_TICK_US)
#define TIMER0_RELOAD       (65536UL - (FOSC_HZ / TIMER0_TICKS_PER_SECOND))

/*
  EEPROM format v2:
    byte 0: magic
    byte 1: WPM
    byte 2: tone index:
            0  = buzzer disabled during normal work
            10 = 1000 Hz
            25 = 2500 Hz
    byte 3: paddle layout
            0 = right hand: P5.5 DASH, P5.4 DOT
            1 = left hand:  P5.5 DOT,  P5.4 DASH
    byte 4: checksum

  Old formats are supported:
    0xC5 -> old WPM only format
    0xC6 -> previous WPM + tone format
*/
#define EEPROM_MAGIC_OLD    0xC5
#define EEPROM_MAGIC_V1     0xC6
#define EEPROM_MAGIC        0xC7

#define EEPROM_ADDR_MAGIC   0x0000
#define EEPROM_ADDR_WPM     0x0001
#define EEPROM_ADDR_TONE    0x0002
#define EEPROM_ADDR_LAYOUT  0x0003
#define EEPROM_ADDR_CHECK   0x0004

#define EEPROM_SAVE_DELAY_MS 600

#define PADDLE_LAYOUT_RIGHT 0
#define PADDLE_LAYOUT_LEFT  1

/*
  Automatic messages.
  Stored in Flash/code memory, not in RAM.

  Morse format:
    .  -> dot
    -  -> dash
    space -> letter gap
    / -> word gap
*/

/*
  CQ CQ DE EW6ML EW6ML EW6ML K
*/
const char code MSG_CQ[] =
    "-.-. --.-/"
    "-.-. --.-/"
    "-.. ./"
    ". .-- -.... -- .-../"
    ". .-- -.... -- .-../"
    ". .-- -.... -- .-../"
    "-.-";

/*
  TNX QSO 73

  T = -
  N = -.
  X = -..-

  Q = --.-
  S = ...
  O = ---

  7 = --...
  3 = ...--
*/
const char code MSG_73[] =
    "- -. -..-/"
    "--.- ... ---/"
    "--... ...--";

/* STC special function registers */
sfr AUXR = 0x8E;

sfr P3M1 = 0xB1;
sfr P3M0 = 0xB2;

sfr P5   = 0xC8;
sfr P5M1 = 0xC9;
sfr P5M0 = 0xCA;

/* STC IAP / EEPROM registers */
sfr IAP_DATA  = 0xC2;
sfr IAP_ADDRH = 0xC3;
sfr IAP_ADDRL = 0xC4;
sfr IAP_CMD   = 0xC5;
sfr IAP_TRIG  = 0xC6;
sfr IAP_CONTR = 0xC7;

#define ENABLE_IAP          0x82
#define CMD_IAP_IDLE        0
#define CMD_IAP_READ        1
#define CMD_IAP_PROGRAM     2
#define CMD_IAP_ERASE       3

/* Pins */
sbit PIEZO_OUT = P3^2;
sbit KEY_OUT   = P3^3;

sbit DOWN_IN = P3^0;
sbit UP_IN   = P3^1;

sbit P54_IN  = P5^4;
sbit P55_IN  = P5^5;

volatile u32 g_ms = 0;
volatile u8 toneEnabled = 0;
volatile u8 tick100usCounter = 0;

/*
  Tone generator.
  Timer tick = 10000 Hz.
*/
volatile u16 toneAcc = 0;

u8 speedWpm = DEFAULT_WPM;
u16 toneHz = DEFAULT_TONE_HZ;
u8 paddleLayout = PADDLE_LAYOUT_RIGHT;

u8 eepromSavePending = 0;
u32 lastSettingChangeMs = 0;

/*
  Physical debounced paddle states.
*/
u8 p54Stable = 0;
u8 p55Stable = 0;

u8 p54LastRaw = 0;
u8 p55LastRaw = 0;

u32 p54LastChangeMs = 0;
u32 p55LastChangeMs = 0;

/*
  Logical debounced paddle states.
  These depend on paddleLayout.
*/
u8 dotStable = 0;
u8 dashStable = 0;

/* Debounced UP/DOWN button states */
u8 downStable = 0;
u8 upStable = 0;

u8 downLastRaw = 0;
u8 upLastRaw = 0;

u32 downLastChangeMs = 0;
u32 upLastChangeMs = 0;

typedef enum {
    STATE_IDLE = 0,
    STATE_DOT_ON,
    STATE_DOT_GAP,
    STATE_DASH_ON,
    STATE_DASH_GAP
} KeyerState;

KeyerState keyerState = STATE_IDLE;
u32 keyerUntilMs = 0;

typedef enum {
    ADJUST_STATE_IDLE = 0,
    ADJUST_STATE_TONE_ON,
    ADJUST_STATE_TONE_GAP
} AdjustState;

typedef enum {
    ADJUST_MODE_SPEED = 0,
    ADJUST_MODE_TONE = 1
} AdjustMode;

AdjustState adjustState = ADJUST_STATE_IDLE;
AdjustMode adjustMode = ADJUST_MODE_SPEED;

u32 adjustUntilMs = 0;
s8 adjustDirection = 0;

typedef enum {
    AUTO_STATE_IDLE = 0,
    AUTO_STATE_ELEMENT_ON,
    AUTO_STATE_ELEMENT_GAP,
    AUTO_STATE_LETTER_GAP,
    AUTO_STATE_WORD_GAP
} AutoState;

AutoState autoState = AUTO_STATE_IDLE;
const char code *autoMsgPtr = 0;
u32 autoUntilMs = 0;

/* ---------- Function prototypes ---------- */

void schedule_settings_save(void);
void save_settings_to_eeprom(void);
void handle_eeprom_save(void);

void tone_on(void);
void tone_off(void);
void key_up(void);
void key_down(void);
void stop_keyer_keep_tone(void);

void auto_message_start(const char code *msg);
void auto_message_stop(void);
void handle_auto_message(void);

void update_buttons(void);

/* ---------- Time and tone ---------- */

u16 get_dot_ms(void) {
    return (u16)(1200 / speedWpm);
}

u16 get_dash_ms(void) {
    return (u16)(get_dot_ms() * 3);
}

u16 get_gap_ms(void) {
    return get_dot_ms();
}

void timer0_load(void) {
    TH0 = (u8)(TIMER0_RELOAD >> 8);
    TL0 = (u8)(TIMER0_RELOAD & 0xFF);
}

void timer0_init_100us(void) {
    TMOD &= 0xF0;
    TMOD |= 0x01;

    /*
      AUXR bit7 = 1 -> Timer0 works in 1T mode on STC15.
    */
    AUXR |= 0x80;

    timer0_load();

    ET0 = 1;
    TR0 = 1;
}

void timer0_isr(void) interrupt 1 {
    timer0_load();

    /*
      100 us tick.
      10 ticks = 1 ms.
    */
    tick100usCounter++;

    if (tick100usCounter >= 10) {
        tick100usCounter = 0;
        g_ms++;
    }

    if (toneEnabled && toneHz > 0) {
        toneAcc += (u16)(toneHz * 2);

        if (toneAcc >= (u16)TIMER0_TICKS_PER_SECOND) {
            toneAcc -= (u16)TIMER0_TICKS_PER_SECOND;
            PIEZO_OUT = !PIEZO_OUT;
        }
    } else {
        toneAcc = 0;
        PIEZO_OUT = 0;
    }
}

u32 millis(void) {
    u32 value;

    EA = 0;
    value = g_ms;
    EA = 1;

    return value;
}

void delay_ms_blocking(u16 ms) {
    u32 start = millis();

    while ((millis() - start) < ms) {
    }
}

void tone_off(void) {
    toneEnabled = 0;
    toneAcc = 0;
    PIEZO_OUT = 0;
}

void tone_on(void) {
    if (toneHz == 0) {
        tone_off();
        return;
    }

    toneAcc = 0;
    toneEnabled = 1;
}

void key_down(void) {
    KEY_OUT = 1;
    tone_on();
}

void key_up(void) {
    KEY_OUT = 0;
    tone_off();
}

void key_output_off_only(void) {
    KEY_OUT = 0;
}

/* ---------- EEPROM / IAP ---------- */

void iap_idle(void) {
    IAP_CONTR = 0;
    IAP_CMD = CMD_IAP_IDLE;
    IAP_TRIG = 0;
    IAP_ADDRH = 0x80;
    IAP_ADDRL = 0;
}

u8 iap_read_byte(u16 addr) {
    u8 value;

    IAP_CONTR = ENABLE_IAP;
    IAP_CMD = CMD_IAP_READ;
    IAP_ADDRL = (u8)(addr & 0xFF);
    IAP_ADDRH = (u8)(addr >> 8);

    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();

    value = IAP_DATA;

    iap_idle();

    return value;
}

void iap_program_byte(u16 addr, u8 value) {
    IAP_CONTR = ENABLE_IAP;
    IAP_CMD = CMD_IAP_PROGRAM;
    IAP_ADDRL = (u8)(addr & 0xFF);
    IAP_ADDRH = (u8)(addr >> 8);
    IAP_DATA = value;

    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();

    iap_idle();
}

void iap_erase_sector(u16 addr) {
    IAP_CONTR = ENABLE_IAP;
    IAP_CMD = CMD_IAP_ERASE;
    IAP_ADDRL = (u8)(addr & 0xFF);
    IAP_ADDRH = (u8)(addr >> 8);

    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();

    iap_idle();
}

u8 eeprom_old_checksum(u8 wpm) {
    return (u8)(EEPROM_MAGIC_OLD ^ wpm ^ 0x5A);
}

u8 eeprom_v1_checksum(u8 wpm, u8 toneIndex) {
    return (u8)(EEPROM_MAGIC_V1 ^ wpm ^ toneIndex ^ 0x5A);
}

u8 eeprom_checksum(u8 wpm, u8 toneIndex, u8 layout) {
    return (u8)(EEPROM_MAGIC ^ wpm ^ toneIndex ^ layout ^ 0x5A);
}

u8 tone_hz_to_index(u16 hz) {
    return (u8)(hz / 100);
}

u16 tone_index_to_hz(u8 index) {
    return (u16)index * 100;
}

void load_settings_from_eeprom(void) {
    u8 magic;
    u8 savedWpm;
    u8 savedToneIndex;
    u8 byte3;
    u8 byte4;

    magic = iap_read_byte(EEPROM_ADDR_MAGIC);
    savedWpm = iap_read_byte(EEPROM_ADDR_WPM);
    savedToneIndex = iap_read_byte(EEPROM_ADDR_TONE);
    byte3 = iap_read_byte(EEPROM_ADDR_LAYOUT);
    byte4 = iap_read_byte(EEPROM_ADDR_CHECK);

    /*
      New v2 format:
        byte3 = layout
        byte4 = checksum
    */
    if (magic == EEPROM_MAGIC &&
        savedWpm >= MIN_WPM &&
        savedWpm <= MAX_WPM &&
        savedToneIndex <= (MAX_TONE_HZ / 100) &&
        byte3 <= PADDLE_LAYOUT_LEFT &&
        byte4 == eeprom_checksum(savedWpm, savedToneIndex, byte3)) {

        speedWpm = savedWpm;
        toneHz = tone_index_to_hz(savedToneIndex);
        paddleLayout = byte3;

        if (toneHz == 0) {
            toneHz = DEFAULT_TONE_HZ;
            schedule_settings_save();
        }

        return;
    }

    /*
      Previous v1 format:
        byte3 = checksum
        layout did not exist
    */
    if (magic == EEPROM_MAGIC_V1 &&
        savedWpm >= MIN_WPM &&
        savedWpm <= MAX_WPM &&
        savedToneIndex <= (MAX_TONE_HZ / 100) &&
        byte3 == eeprom_v1_checksum(savedWpm, savedToneIndex)) {

        speedWpm = savedWpm;
        toneHz = tone_index_to_hz(savedToneIndex);
        paddleLayout = PADDLE_LAYOUT_RIGHT;

        if (toneHz == 0) {
            toneHz = DEFAULT_TONE_HZ;
        }

        schedule_settings_save();
        return;
    }

    /*
      Old format compatibility:
        byte2 = checksum
        tone did not exist
        layout did not exist
    */
    if (magic == EEPROM_MAGIC_OLD &&
        savedWpm >= MIN_WPM &&
        savedWpm <= MAX_WPM &&
        savedToneIndex == eeprom_old_checksum(savedWpm)) {

        speedWpm = savedWpm;
        toneHz = DEFAULT_TONE_HZ;
        paddleLayout = PADDLE_LAYOUT_RIGHT;

        schedule_settings_save();
        return;
    }

    speedWpm = DEFAULT_WPM;
    toneHz = DEFAULT_TONE_HZ;
    paddleLayout = PADDLE_LAYOUT_RIGHT;
}

void schedule_settings_save(void) {
    eepromSavePending = 1;
    lastSettingChangeMs = millis();
}

void save_settings_to_eeprom(void) {
    u8 toneIndex;

    toneIndex = tone_hz_to_index(toneHz);

    /*
      EEPROM sector is erased before writing.
    */
    iap_erase_sector(EEPROM_ADDR_MAGIC);

    iap_program_byte(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
    iap_program_byte(EEPROM_ADDR_WPM, speedWpm);
    iap_program_byte(EEPROM_ADDR_TONE, toneIndex);
    iap_program_byte(EEPROM_ADDR_LAYOUT, paddleLayout);
    iap_program_byte(EEPROM_ADDR_CHECK, eeprom_checksum(speedWpm, toneIndex, paddleLayout));
}

void handle_eeprom_save(void) {
    if (!eepromSavePending) {
        return;
    }

    /*
      Do not write while adjustment buttons are still held.
      This reduces EEPROM wear during long button holds.
    */
    if (downStable || upStable) {
        return;
    }

    if ((millis() - lastSettingChangeMs) >= EEPROM_SAVE_DELAY_MS) {
        save_settings_to_eeprom();
        eepromSavePending = 0;
    }
}

/* ---------- Pins ---------- */

void pins_init(void) {
    /*
      P3.2 -> piezo output, push-pull
      P3.3 -> key output, push-pull

      P3.0 -> DOWN input
      P3.1 -> UP input

      P5.4 -> paddle input
      P5.5 -> paddle input
    */

    /* P3.2 and P3.3 push-pull outputs */
    P3M0 |= 0x0C;
    P3M1 &= ~0x0C;

    /*
      P3.0 and P3.1 as quasi-bidirectional inputs with weak pull-up.
    */
    P3M0 &= ~0x03;
    P3M1 &= ~0x03;
    P3 |= 0x03;

    /*
      P5.4 and P5.5 as quasi-bidirectional inputs with weak pull-up.
    */
    P5M0 &= ~0x30;
    P5M1 &= ~0x30;
    P5 |= 0x30;

    KEY_OUT = 0;
    PIEZO_OUT = 0;
}

/* ---------- Buttons ---------- */

void update_buttons(void) {
    u32 now = millis();

    u8 p54Raw;
    u8 p55Raw;
    u8 downRaw;
    u8 upRaw;

    p54Raw = (P54_IN == 0) ? 1 : 0;
    p55Raw = (P55_IN == 0) ? 1 : 0;

    downRaw = (DOWN_IN == 0) ? 1 : 0;
    upRaw = (UP_IN == 0) ? 1 : 0;

    if (p54Raw != p54LastRaw) {
        p54LastRaw = p54Raw;
        p54LastChangeMs = now;
    }

    if ((now - p54LastChangeMs) >= DEBOUNCE_MS) {
        p54Stable = p54Raw;
    }

    if (p55Raw != p55LastRaw) {
        p55LastRaw = p55Raw;
        p55LastChangeMs = now;
    }

    if ((now - p55LastChangeMs) >= DEBOUNCE_MS) {
        p55Stable = p55Raw;
    }

    if (downRaw != downLastRaw) {
        downLastRaw = downRaw;
        downLastChangeMs = now;
    }

    if ((now - downLastChangeMs) >= DEBOUNCE_MS) {
        downStable = downRaw;
    }

    if (upRaw != upLastRaw) {
        upLastRaw = upRaw;
        upLastChangeMs = now;
    }

    if ((now - upLastChangeMs) >= DEBOUNCE_MS) {
        upStable = upRaw;
    }

    /*
      Convert physical paddle buttons to logical DOT/DASH.

      RIGHT layout:
        P5.5 = DASH
        P5.4 = DOT

      LEFT layout:
        P5.5 = DOT
        P5.4 = DASH
    */
    if (paddleLayout == PADDLE_LAYOUT_LEFT) {
        dotStable = p55Stable;
        dashStable = p54Stable;
    } else {
        dotStable = p54Stable;
        dashStable = p55Stable;
    }
}

/* ---------- Startup greeting ---------- */

void send_startup_tone_dot(void) {
    tone_on();
    delay_ms_blocking(get_dot_ms());
    tone_off();
    delay_ms_blocking(get_gap_ms());
}

void send_startup_hi(void) {
    u8 i;

    /*
      Startup greeting: "HI"
      H = ....
      I = ..

      Sidetone only.
      KEY_OUT is not activated.
    */
    KEY_OUT = 0;

    delay_ms_blocking(300);

    /* H */
    for (i = 0; i < 4; i++) {
        send_startup_tone_dot();
    }

    /*
      Letter gap.
      One element gap is already included after the last dot,
      so add 2 more dot lengths.
    */
    delay_ms_blocking(get_dot_ms() * 2);

    /* I */
    for (i = 0; i < 2; i++) {
        send_startup_tone_dot();
    }

    tone_off();
    KEY_OUT = 0;

    delay_ms_blocking(300);
}

/* ---------- Paddle layout switching ---------- */

void play_layout_confirm_tone(void) {
    /*
      Confirmation tone only.
      KEY output must stay inactive.
    */
    KEY_OUT = 0;

    tone_on();
    delay_ms_blocking(1000);
    tone_off();

    KEY_OUT = 0;
}

void set_paddle_layout(u8 layout) {
    if (layout > PADDLE_LAYOUT_LEFT) {
        return;
    }

    if (paddleLayout != layout) {
        paddleLayout = layout;
        save_settings_to_eeprom();
        eepromSavePending = 0;
    }

    play_layout_confirm_tone();
}

/* ---------- Automatic CW messages ---------- */

void auto_message_stop(void) {
    key_up();

    autoState = AUTO_STATE_IDLE;
    autoMsgPtr = 0;
    autoUntilMs = 0;
}

void auto_message_next_symbol(void) {
    char c;
    u32 now;

    if (autoMsgPtr == 0) {
        auto_message_stop();
        return;
    }

    while (1) {
        now = millis();
        c = *autoMsgPtr;

        if (c == 0) {
            auto_message_stop();
            return;
        }

        autoMsgPtr++;

        if (c == '.') {
            key_down();
            autoState = AUTO_STATE_ELEMENT_ON;
            autoUntilMs = now + get_dot_ms();
            return;
        }

        if (c == '-') {
            key_down();
            autoState = AUTO_STATE_ELEMENT_ON;
            autoUntilMs = now + get_dash_ms();
            return;
        }

        if (c == ' ') {
            /*
              After each element we already add 1 dot gap.
              Letter gap must be 3 dots total, so add 2 more dots.
            */
            key_up();
            autoState = AUTO_STATE_LETTER_GAP;
            autoUntilMs = now + ((u32)get_dot_ms() * 2);
            return;
        }

        if (c == '/') {
            /*
              After each element we already add 1 dot gap.
              Word gap must be 7 dots total, so add 6 more dots.
            */
            key_up();
            autoState = AUTO_STATE_WORD_GAP;
            autoUntilMs = now + ((u32)get_dot_ms() * 6);
            return;
        }

        /*
          Unknown character: skip it and read next symbol.
          No recursion is used here.
        */
    }
}

void auto_message_start(const char code *msg) {
    key_up();

    keyerState = STATE_IDLE;
    adjustState = ADJUST_STATE_IDLE;
    adjustDirection = 0;

    autoMsgPtr = msg;
    autoState = AUTO_STATE_IDLE;

    auto_message_next_symbol();
}

void handle_auto_message(void) {
    u32 now = millis();

    if (autoState == AUTO_STATE_IDLE) {
        return;
    }

    /*
      Manual paddle press cancels automatic message.
      UP/DOWN buttons do not cancel it by short touch.
    */
    if (dotStable || dashStable) {
        auto_message_stop();
        return;
    }

    switch (autoState) {
        case AUTO_STATE_ELEMENT_ON:
            if ((long)(now - autoUntilMs) >= 0) {
                key_up();

                autoState = AUTO_STATE_ELEMENT_GAP;
                autoUntilMs = now + get_gap_ms();
            }
            break;

        case AUTO_STATE_ELEMENT_GAP:
        case AUTO_STATE_LETTER_GAP:
        case AUTO_STATE_WORD_GAP:
            if ((long)(now - autoUntilMs) >= 0) {
                auto_message_next_symbol();
            }
            break;

        default:
            auto_message_stop();
            break;
    }
}

/* ---------- CW keyer ---------- */

void start_dot(void) {
    key_down();

    keyerState = STATE_DOT_ON;
    keyerUntilMs = millis() + get_dot_ms();
}

void start_dash(void) {
    key_down();

    keyerState = STATE_DASH_ON;
    keyerUntilMs = millis() + get_dash_ms();
}

void stop_keyer(void) {
    key_up();
    keyerState = STATE_IDLE;
}

void stop_keyer_keep_tone(void) {
    /*
      Used during adjustment.
      KEY_OUT must be inactive, but sidetone must not be interrupted.
    */
    key_output_off_only();
    keyerState = STATE_IDLE;
}

void handle_keyer(void) {
    u32 now = millis();

    /*
      Automatic message has priority over manual keyer.
    */
    if (autoState != AUTO_STATE_IDLE) {
        return;
    }

    /*
      Adjustment mode has priority.
      During adjustment, KEY_OUT must stay inactive.
      Important: do not disable sidetone here,
      because adjustment buttons use sidetone feedback.
    */
    if (adjustState != ADJUST_STATE_IDLE || downStable || upStable) {
        stop_keyer_keep_tone();
        return;
    }

    switch (keyerState) {
        case STATE_IDLE:
            /*
              Dash has priority if both paddles are pressed.
            */
            if (dashStable) {
                start_dash();
            } else if (dotStable) {
                start_dot();
            }
            break;

        case STATE_DOT_ON:
            if ((long)(now - keyerUntilMs) >= 0) {
                key_up();

                keyerState = STATE_DOT_GAP;
                keyerUntilMs = now + get_gap_ms();
            }
            break;

        case STATE_DOT_GAP:
            if ((long)(now - keyerUntilMs) >= 0) {
                if (dashStable) {
                    start_dash();
                } else if (dotStable) {
                    start_dot();
                } else {
                    keyerState = STATE_IDLE;
                }
            }
            break;

        case STATE_DASH_ON:
            if ((long)(now - keyerUntilMs) >= 0) {
                key_up();

                keyerState = STATE_DASH_GAP;
                keyerUntilMs = now + get_gap_ms();
            }
            break;

        case STATE_DASH_GAP:
            if ((long)(now - keyerUntilMs) >= 0) {
                if (dashStable) {
                    start_dash();
                } else if (dotStable) {
                    start_dot();
                } else {
                    keyerState = STATE_IDLE;
                }
            }
            break;

        default:
            stop_keyer();
            break;
    }
}

/* ---------- Adjustment: speed and tone ---------- */

s8 get_adjust_direction(void) {
    if (upStable && !downStable) {
        return 1;
    }

    if (downStable && !upStable) {
        return -1;
    }

    return 0;
}

AdjustMode get_adjust_mode(void) {
    /*
      If physical P5.5 is held while pressing UP/DOWN,
      adjust sidetone frequency instead of WPM.
    */
    if (p55Stable) {
        return ADJUST_MODE_TONE;
    }

    return ADJUST_MODE_SPEED;
}

void change_speed(s8 direction) {
    if (direction > 0) {
        if (speedWpm < MAX_WPM) {
            speedWpm++;
            schedule_settings_save();
        }
    } else if (direction < 0) {
        if (speedWpm > MIN_WPM) {
            speedWpm--;
            schedule_settings_save();
        }
    }
}

void change_tone(s8 direction) {
    if (direction > 0) {
        if (toneHz < MAX_TONE_HZ) {
            toneHz += TONE_STEP_HZ;

            if (toneHz > MAX_TONE_HZ) {
                toneHz = MAX_TONE_HZ;
            }

            schedule_settings_save();
        }
    } else if (direction < 0) {
        if (toneHz > MIN_TONE_HZ) {
            if (toneHz >= TONE_STEP_HZ) {
                toneHz -= TONE_STEP_HZ;
            } else {
                toneHz = MIN_TONE_HZ;
            }

            schedule_settings_save();
        }
    }
}

void apply_adjustment(s8 direction, AdjustMode mode) {
    if (mode == ADJUST_MODE_TONE) {
        change_tone(direction);
    } else {
        change_speed(direction);
    }
}

void start_adjust_tone_dot(s8 direction) {
    /*
      Adjustment feedback:
        - speed mode: change WPM by one step
        - tone mode: change tone by 100 Hz

      Then play one dot using sidetone only.
      KEY_OUT stays inactive.
    */
    KEY_OUT = 0;

    adjustDirection = direction;
    adjustMode = get_adjust_mode();

    apply_adjustment(direction, adjustMode);

    tone_on();

    adjustState = ADJUST_STATE_TONE_ON;
    adjustUntilMs = millis() + get_dot_ms();
}

void start_adjust_speed_dot(s8 direction) {
    /*
      Speed adjustment feedback.
      Used only after long UP/DOWN press.
    */
    KEY_OUT = 0;

    adjustDirection = direction;
    adjustMode = ADJUST_MODE_SPEED;

    apply_adjustment(direction, adjustMode);

    tone_on();

    adjustState = ADJUST_STATE_TONE_ON;
    adjustUntilMs = millis() + get_dot_ms();
}

void wait_layout_buttons_release(void) {
    /*
      Prevent repeated switching while buttons are still held.
    */
    while (p54Stable || downStable || upStable) {
        update_buttons();
    }
}

void handle_adjust_buttons(void) {
    u32 now = millis();
    s8 direction;

    static u8 activeButton = 0;
    static u8 longMode = 0;
    static u32 pressStartMs = 0;

    /*
      activeButton:
        0 = no UP/DOWN button is being tracked
        1 = DOWN
        2 = UP
    */

    /*
      Paddle layout switching.

      Hold physical P5.4 and press:
        P3.0 -> left-hand layout
        P3.1 -> right-hand layout

      This mode has priority over speed/tone adjustment.
    */
    if (p54Stable && (downStable || upStable)) {
        auto_message_stop();
        stop_keyer_keep_tone();

        adjustState = ADJUST_STATE_IDLE;
        adjustDirection = 0;

        activeButton = 0;
        longMode = 0;

        if (downStable && !upStable) {
            set_paddle_layout(PADDLE_LAYOUT_LEFT);
            wait_layout_buttons_release();
            return;
        }

        if (upStable && !downStable) {
            set_paddle_layout(PADDLE_LAYOUT_RIGHT);
            wait_layout_buttons_release();
            return;
        }

        return;
    }

    /*
      Tone adjustment mode:
      Hold physical P5.5 and press UP/DOWN.
    */
    if (p55Stable) {
        activeButton = 0;
        longMode = 0;

        switch (adjustState) {
            case ADJUST_STATE_IDLE:
                direction = get_adjust_direction();

                if (direction != 0) {
                    auto_message_stop();
                    stop_keyer_keep_tone();
                    start_adjust_tone_dot(direction);
                }
                break;

            case ADJUST_STATE_TONE_ON:
                if ((long)(now - adjustUntilMs) >= 0) {
                    tone_off();
                    KEY_OUT = 0;

                    adjustState = ADJUST_STATE_TONE_GAP;
                    adjustUntilMs = now + get_gap_ms();
                }
                break;

            case ADJUST_STATE_TONE_GAP:
                if ((long)(now - adjustUntilMs) >= 0) {
                    direction = get_adjust_direction();

                    if (direction != 0) {
                        start_adjust_tone_dot(direction);
                    } else {
                        tone_off();
                        KEY_OUT = 0;
                        adjustDirection = 0;
                        adjustState = ADJUST_STATE_IDLE;
                    }
                }
                break;

            default:
                tone_off();
                KEY_OUT = 0;
                adjustDirection = 0;
                adjustState = ADJUST_STATE_IDLE;
                break;
        }

        return;
    }

    /*
      If both UP and DOWN are pressed, do nothing.
      This prevents accidental speed changes.
    */
    if (downStable && upStable) {
        return;
    }

    /*
      Start tracking a new button press.
      Important: no speed change here.
    */
    if (activeButton == 0) {
        if (downStable) {
            activeButton = 1;
            longMode = 0;
            pressStartMs = now;
        } else if (upStable) {
            activeButton = 2;
            longMode = 0;
            pressStartMs = now;
        } else {
            return;
        }
    }

    /*
      DOWN is being tracked.
    */
    if (activeButton == 1) {
        /*
          Button released before long press threshold:
          short press -> CQ message.
        */
        if (!downStable) {
            if (!longMode) {
                if (autoState == AUTO_STATE_IDLE && adjustState == ADJUST_STATE_IDLE) {
                    auto_message_start(MSG_CQ);
                }
            } else {
                tone_off();
                KEY_OUT = 0;
                adjustDirection = 0;
                adjustState = ADJUST_STATE_IDLE;
            }

            activeButton = 0;
            longMode = 0;
            return;
        }

        /*
          Button is still held.
          Enable speed adjustment only after LONG_PRESS_MS.
        */
        if (!longMode) {
            if ((now - pressStartMs) < LONG_PRESS_MS) {
                return;
            }

            longMode = 1;
            auto_message_stop();
            stop_keyer_keep_tone();
        }

        direction = -1;
    }

    /*
      UP is being tracked.
    */
    else if (activeButton == 2) {
        /*
          Button released before long press threshold:
          short press -> TNX QSO 73 message.
        */
        if (!upStable) {
            if (!longMode) {
                if (autoState == AUTO_STATE_IDLE && adjustState == ADJUST_STATE_IDLE) {
                    auto_message_start(MSG_73);
                }
            } else {
                tone_off();
                KEY_OUT = 0;
                adjustDirection = 0;
                adjustState = ADJUST_STATE_IDLE;
            }

            activeButton = 0;
            longMode = 0;
            return;
        }

        /*
          Button is still held.
          Enable speed adjustment only after LONG_PRESS_MS.
        */
        if (!longMode) {
            if ((now - pressStartMs) < LONG_PRESS_MS) {
                return;
            }

            longMode = 1;
            auto_message_stop();
            stop_keyer_keep_tone();
        }

        direction = 1;
    } else {
        activeButton = 0;
        longMode = 0;
        return;
    }

    /*
      Long press mode:
      now and only now UP/DOWN changes WPM.
    */
    switch (adjustState) {
        case ADJUST_STATE_IDLE:
            start_adjust_speed_dot(direction);
            break;

        case ADJUST_STATE_TONE_ON:
            if ((long)(now - adjustUntilMs) >= 0) {
                tone_off();
                KEY_OUT = 0;

                adjustState = ADJUST_STATE_TONE_GAP;
                adjustUntilMs = now + get_gap_ms();
            }
            break;

        case ADJUST_STATE_TONE_GAP:
            if ((long)(now - adjustUntilMs) >= 0) {
                start_adjust_speed_dot(direction);
            }
            break;

        default:
            tone_off();
            KEY_OUT = 0;
            adjustDirection = 0;
            adjustState = ADJUST_STATE_IDLE;
            break;
    }
}

/* ---------- Main ---------- */

void main(void) {
    pins_init();

    timer0_init_100us();

    EA = 1;

    load_settings_from_eeprom();

    send_startup_hi();

    while (1) {
        update_buttons();

        handle_adjust_buttons();
        handle_auto_message();
        handle_keyer();
        handle_eeprom_save();
    }
}