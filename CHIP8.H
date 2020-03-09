#define MEM_SIZE                4096  /* 0x1000 */
#define ADDRESS_MASK            (MEM_SIZE - 1)
/* #define CHECK_PC_LIMITS           1 */  /* if defined, make sure PC "wraps" above MEM_SIZE */
#define PROGRAM_START         0x0200
#define ETI660_PROGRAM_START  0x0600
#define MIN_PROGRAM_SIZE           2   /* need at least 1 instruction! */
#define MAX_PC                (MEM_SIZE - 2)  /* instructions are 2 bytes long */
#define STACK_SIZE                16
#define STACK_START            0x1e0

#define VF_REG                    15
#define NUM_REGISTERS             16

#define X_REG_OPCODE_FIELD      opcode_high & 0x0F
#define Y_REG_OPCODE_FIELD      opcode_low >> 4

#define HEX_SPRITES_LOCATION   0x080
#define HEX_SPRITES_ROWS           5
#define HEX_SPRITES_NUM_BYTES (16 * HEX_SPRITES_ROWS)
#define HEX_SPRITES_SUPER_ROWS   10
#define HEX_SPRITES_SUPER_LOCATION  (HEX_SPRITES_LOCATION + HEX_SPRITES_NUM_BYTES)
#define HEX_SPRITES_SUPER_NUM_BYTES (16 * HEX_SPRITES_SUPER_ROWS)

#define GRAPHICS_ROWS            64
#define GRAPHICS_COLS           128
#define LOWRES_GRAPHICS_ROWS     32
#define LOWRES_GRAPHICS_COLS     64


#define GRAPHICS_MODE        0x13  /* 200 (rows) x 320 (cols) x 8bits/pixel */
                                   /* at A000:0000 */
#define GRAPHICS_SEG       0xA000
#define TEXT_MODE            0x03  /* standard text */
#define BLACK                0x00
#define CYAN                 0x03
#define WHITE                0x0F

#define MIN_SOUND_FREQUENCY      100
#define MAX_SOUND_FREQUENCY     1000
#define DEFAULT_SOUND_FREQUENCY  330

#define SYSTEM_CLOCK_TICK_SEG  0
#define SYSTEM_CLOCK_TICK_OFS  0x046c

#define TIMER_CLOCK_BASE   1193182  /* 8253 input frequency */

#define CHIP8_EXECUTION_TICKS_PER_SECOND      2040
#define CHIP8_EXECUTION_TICKS_TO_60HZ         ((CHIP8_EXECUTION_TICKS_PER_SECOND + 30) / 60)
#define CHIP8_EXECUTION_TICKS_8253_DIVISOR    ((TIMER_CLOCK_BASE + (CHIP8_EXECUTION_TICKS_PER_SECOND / 2)) / CHIP8_EXECUTION_TICKS_PER_SECOND)
#define CHIP8_EXECUTION_TICKS_TO_CLK_TICKS    (((CHIP8_EXECUTION_TICKS_PER_SECOND * 10) + 91) / 182)
/* Why 2040?  It's because I have 3 timer values to generate from a single
 * source.
 *
 *   1) I want about 2K interrupts per second to pace the interpreter
 *   2) I need a 60Hz timer
 *   3) I need a 18.2Hz timer for the system clock tick.
 *
 *  Doing integer math, I get:
 *   1193182 / 2040 == 584.89 ~= 585        for the 8253 divisor
 *   1193182 / 585 == 2039.63               for the actual interrupt frequency
 *   2039.63 / 60 == 33.99 ~= 34            for the 60Hz divisor
 *   2039.63 / 18.2 == 112.07 ~= 112        for the 18.2Hz divisor
 */

/* CHIP8 expects a hex keypad in the following format:
 *
 *   +----+----+----+----+
 *   |    |    |    |    |
 *   | 1  | 2  | 3  | C  |
 *   |    |    |    |    |
 *   +----+----+----+----+
 *   |    |    |    |    |
 *   | 4  | 5  | 6  | D  |
 *   |    |    |    |    |
 *   +----+----+----+----+
 *   |    |    |    |    |
 *   | 7  | 8  | 9  | E  |
 *   |    |    |    |    |
 *   +----+----+----+----+
 *   |    |    |    |    |
 *   | A  | 0  | B  | F  |
 *   |    |    |    |    |
 *   +----+----+----+----+
 *
 *  This program will map the PC keyboard keys like this:
 *
 *   +----+----+----+----+
 *   |    |    |    |    |
 *   |Num | C  | D  | E  |
 *   |Lk  |    |    |    |
 *   +----+----+----+----+
 *   |    |    |    |    |
 *   | 1  | 2  | 3  | F  |
 *   |    |    |    |    |
 *   +----+----+----+    +
 *   |    |    |    |    |
 *   | 4  | 5  | 6  |    |
 *   |    |    |    |    |
 *   +----+----+----+----+
 *   |    |    |    |    |
 *   | 7  | 8  | 9  | B  |
 *   |    |    |    |    |
 *   +----+----+----+    +
 *   |         |    |    |
 *   | A       | 0  |    |
 *   |         |    |    |
 *   +----+----+----+----+
 */
 /*  Note: These values are read from PORT 60H when a key is pressed */
#define ALT_HEX_0_KEY   0x2D
#define ALT_HEX_1_KEY   0x02
#define ALT_HEX_2_KEY   0x03
#define ALT_HEX_3_KEY   0x04
#define ALT_HEX_4_KEY   0x10
#define ALT_HEX_5_KEY   0x11
#define ALT_HEX_6_KEY   0x12
#define ALT_HEX_7_KEY   0x1E
#define ALT_HEX_8_KEY   0x1F
#define ALT_HEX_9_KEY   0x20
#define ALT_HEX_A_KEY   0x2C
#define ALT_HEX_B_KEY   0x2E
#define ALT_HEX_C_KEY   0x05
#define ALT_HEX_D_KEY   0x13
#define ALT_HEX_E_KEY   0x21
#define ALT_HEX_F_KEY   0x2F

/*  An alternate mapping will be:
 *
 *   +----+----+----+----+
 *   | 1  | 2  | 3  | 4  |
 *   |    |    |    | -> |
 *   |    |    |    | C  |
 *   +----+----+----+----+
 *   | q  | w  | e  | r  |
 *   | -> | -> | -> | -> |
 *   | 4  | 5  | 6  | D  |
 *   +----+----+----+----+
 *   | a  | s  | d  | f  |
 *   | -> | -> | -> | -> |
 *   | 7  | 8  | 9  | E  |
 *   +----+----+----+----+
 *   | z  | x  | c  | v  |
 *   | -> | -> | -> | -> |
 *   | A  | 0  | B  | F  |
 *   +----+----+----+----+
 */
 /*  Note: These values are read from PORT 60H when a key is pressed */
#define HEX_0_KEY  0x53
#define HEX_1_KEY  0x47
#define HEX_2_KEY  0x48
#define HEX_3_KEY  0x49
#define HEX_4_KEY  0x4B
#define HEX_5_KEY  0x4C
#define HEX_6_KEY  0x4D
#define HEX_7_KEY  0x4F
#define HEX_8_KEY  0x50
#define HEX_9_KEY  0x51
#define HEX_A_KEY  0x52
#define HEX_B_KEY  0x1C
#define HEX_C_KEY  0x35
#define HEX_D_KEY  0x37
#define HEX_E_KEY  0x4A
#define HEX_F_KEY  0x4E

#define NUM_KEYS    16



#define ESC_KEY_SCANCODE       1
#define ESC_KEY                '\033'


#define ERROR_NONE             0
#define ERROR_EXIT             1
#define ERROR_STACK_OVERFLOW   2
#define ERROR_STACK_UNDERFLOW  3
#define ERROR_UNKNOWN_OPCODE   4
#define ERROR_PC_RANGE         5

#define COUNT_INSTRUCTIONS     1
