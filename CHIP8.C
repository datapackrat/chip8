#include "chip8.h"

typedef unsigned char BYTE;  /* 8-bits, unsigned */
typedef unsigned int  WORD;  /* 16-bits, unsigned */
#ifdef COUNT_INSTRUCTIONS
   typedef long         DWORD;  /* DeSmet does not support 32-bit unsigned */
#endif

#define TRUE 1
#define FALSE 0

BYTE regs[NUM_REGISTERS];  /* V0 .. VF */
BYTE hp48regs[8];  /* may as well implement these, too... */
WORD pc;  /* program counter */
WORD sp = STACK_START;
          /* Note:  Although stack size is 16 entires, it starts at
           *        memory address 0x01E0, and so has to be a WORD
           */
WORD I_reg;   /* I register */
BYTE memory[MEM_SIZE];
   /* Note:  CHIP-8 code is stored BIG-ENDIAN.  Native x86 code is LITTLE-ENDIAN */
WORD program_load_addr = PROGRAM_START;  /* variable, for ETI-660 support */

BYTE video_memory[GRAPHICS_ROWS * GRAPHICS_COLS];
    /* +-----------------------------+
     * | (0,0)           (maxcol, 0) |
     * |                             |
     * |                             |
     * |                             |
     * | (0,maxrow)  (maxcol,maxrow) |
     * +-----------------------------+
     */
BYTE super_chip8_video = FALSE;  /* FALSE == 64 x 32, TRUE == 128 x 64 */


BYTE sound_timer;
BYTE delay_timer;
WORD execution_ticks_counter;
BYTE execution_tick_flag;
BYTE system_clock_count = CHIP8_EXECUTION_TICKS_TO_CLK_TICKS;
BYTE system_clock_divisor = CHIP8_EXECUTION_TICKS_TO_CLK_TICKS;
       /* need CHIP8_EXECUTION_TICKS_TO_CLK_TICKS in a variable for assembly code */
BYTE sixty_hz_divisor = CHIP8_EXECUTION_TICKS_TO_60HZ;
       /* need CHIP8_EXECUTION_TICKS_TO_60HZ in a variable for assembly code */
BYTE sixty_hz_count = CHIP8_EXECUTION_TICKS_TO_60HZ;
BYTE sixty_hz_flag;
WORD sound_frequency = DEFAULT_SOUND_FREQUENCY;

/* following variables are for inline assembly language */
BYTE screen_cols = LOWRES_GRAPHICS_COLS;
BYTE screen_rows = LOWRES_GRAPHICS_ROWS;
BYTE max_screen_cols = LOWRES_GRAPHICS_COLS - 1;
BYTE max_screen_rows = LOWRES_GRAPHICS_ROWS - 1;
WORD video_seg = GRAPHICS_SEG;
BYTE white = WHITE;
BYTE black = BLACK;
BYTE wrap_sprites = FALSE;
       /* according to the documentation, it looks like the DRAW
        * command WRAPS the sprite to the start of the next line
        * if the sprite goes beyond the "right edge" of the screen.
        *
        * However, some CHIP8 programs don't work correctly unless
        * the sprite STOPS at the "right edge" of the screen.
        *
        * The default will be to STOP the sprite at the edge of the
        * screen, but this behavior can be toggled.
        */
BYTE use_vy_shift = FALSE;
   /* The documentation on the 8xyE instruction between various
    * sources does not agree.  Sources like devernay.free.fr
    * and craigthomas.ca/blog say that 8xyE is Vx = Vx << 1, VF = carry
    *
    * However, github.com/mattmikolay/chip-8/wiki/CHIP-8-Instruction-Set
    * says that 8xyE is Vx = Vy << 1, VF = carry
    *
    * The Wikipedia article en.wikipedia.org/wiki/CHIP-8 notes this
    * discrepancy (see the Opcode Table notes).
    */
BYTE i_reg_is_incremented = FALSE;
   /* The documentation on the I register is inconsistent.  Some
    * sources (see above) say that the instructions Fx55 (LD [I], Vx)
    * and Fx65 (LD Vx, [I]) do not increment I, other say it does.
    */
BYTE i_reg_sets_carry = FALSE;
   /* An "undocumented" feature is that the ADD I,Vx instruction will
    * will set VF to CARRY.  The standard sources (see above) don't
    * document this.  It is not the standard behavior because destroying
    * VF can cause problems with programs that don't expect it.
    *
    * Wikipedia (en.wikipedia.org/wiki/CHIP-8) notes that the program
    * Spacefight 2091! uses this "undocumented" feature.
    */

WORD opcode;       /* full opcode, will be used in pieces as needed */
BYTE opcode_high;  /* MSB */
BYTE opcode_main;  /* high nibble of opcode MSB shifted 4 right */
BYTE opcode_x;     /* opcode MSB & 0x0f */
BYTE opcode_low;   /* LSB, also 8-bit constant value */
BYTE opcode_y;     /* high nibble of opcode LSB shifted 4 right */
BYTE opcode_subfn; /* secondary opcode function, etc */
BYTE opcode_n;     /* nibble */
WORD opcode_xxx;   /* opcode word & ADDRESS_MASK --> 12 bit value for ADDR and I reg */

WORD execution_ticks_per_instruction = 1;
WORD instructions_per_execution_tick = 1;
#ifdef COUNT_INSTRUCTIONS
   WORD total_instruction_count[4];
#endif
BYTE execution_freerun = FALSE;

BYTE hex_sprites[HEX_SPRITES_NUM_BYTES] =
   {
   0xF0,  /* 11110000   xxxx   */
   0x90,  /* 10010000   x  x   */
   0x90,  /* 10010000   x  x   */
   0x90,  /* 10010000   x  x   */
   0xF0,  /* 11110000   xxxx   */

   0x20,  /* 00100000     x    */
   0x60,  /* 01100000    xx    */
   0x20,  /* 00100000     x    */
   0x20,  /* 00100000     x    */
   0x70,  /* 01110000    xxx   */

   0xF0,  /* 11110000   xxxx   */
   0x10,  /* 00010000      x   */
   0xF0,  /* 11110000   xxxx   */
   0x80,  /* 10000000   x      */
   0xF0,  /* 11110000   xxxx   */

   0xF0,  /* 11110000   xxxx   */
   0x10,  /* 00010000      x   */
   0xF0,  /* 11110000   xxxx   */
   0x10,  /* 00010000      x   */
   0xF0,  /* 11110000   xxxx   */

   0x90,  /* 10010000   x  x   */
   0x90,  /* 10010000   x  x   */
   0xF0,  /* 11110000   xxxx   */
   0x10,  /* 00010000      x   */
   0x10,  /* 00010000      x   */

   0xF0,  /* 11110000   xxxx   */
   0x80,  /* 10000000   x      */
   0xF0,  /* 11110000   xxxx   */
   0x10,  /* 00010000      x   */
   0xF0,  /* 11110000   xxxx   */

   0xF0,  /* 11110000   xxxx   */
   0x80,  /* 10000000   x      */
   0xF0,  /* 11110000   xxxx   */
   0x90,  /* 10010000   x  x   */
   0xF0,  /* 11110000   xxxx   */

   0xF0,  /* 11110000   xxxx   */
   0x10,  /* 00010000      x   */
   0x20,  /* 00100000     x    */
   0x40,  /* 01000000    x     */
   0x40,  /* 01000000    x     */

   0xF0,  /* 11110000   xxxx   */
   0x90,  /* 10010000   x  x   */
   0xF0,  /* 11110000   xxxx   */
   0x90,  /* 10010000   x  x   */
   0xF0,  /* 11110000   xxxx   */

   0xF0,  /* 11110000   xxxx   */
   0x90,  /* 10010000   x  x   */
   0xF0,  /* 11110000   xxxx   */
   0x10,  /* 00010000      x   */
   0xF0,  /* 11110000   xxxx   */

   0xF0,  /* 11110000   xxxx   */
   0x90,  /* 10010000   x  x   */
   0xF0,  /* 11110000   xxxx   */
   0x90,  /* 10010000   x  x   */
   0x90,  /* 10010000   x  x   */

   0xE0,  /* 11100000   xxx    */
   0x90,  /* 10010000   x  x   */
   0xE0,  /* 11100000   xxx    */
   0x90,  /* 10010000   x  x   */
   0xE0,  /* 11100000   xxx    */

   0xF0,  /* 11110000   xxxx   */
   0x80,  /* 10000000   x      */
   0x80,  /* 10000000   x      */
   0x80,  /* 10000000   x      */
   0xF0,  /* 11110000   xxxx   */

   0xE0,  /* 11100000   xxx    */
   0x90,  /* 10010000   x  x   */
   0x90,  /* 10010000   x  x   */
   0x90,  /* 10010000   x  x   */
   0xE0,  /* 11100000   xxx    */

   0xF0,  /* 11110000   xxxx   */
   0x80,  /* 10000000   x      */
   0xF0,  /* 11110000   xxxx   */
   0x80,  /* 10000000   x      */
   0xF0,  /* 11110000   xxxx   */

   0xF0,  /* 11110000   xxxx   */
   0x80,  /* 10000000   x      */
   0xF0,  /* 11110000   xxxx   */
   0x80,  /* 10000000   x      */
   0x80   /* 10000000   x      */
   };

BYTE hex_sprites_super[HEX_SPRITES_SUPER_NUM_BYTES] =
   {
   0x3C,  /* 00111100    xxxx   */
   0x7E,  /* 01111110   xxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0x7E,  /* 01111110   xxxxxx  */
   0x3C,  /* 00111100    xxxx   */

   0x18,  /* 00011000     xx    */
   0x38,  /* 00111000    xxx    */
   0x58,  /* 01011000   x xx    */
   0x18,  /* 00011000     xx    */
   0x18,  /* 00011000     xx    */
   0x18,  /* 00011000     xx    */
   0x18,  /* 00011000     xx    */
   0x18,  /* 00011000     xx    */
   0x18,  /* 00011000     xx    */
   0x3C,  /* 00111100    xxxx   */

   0x3E,  /* 00111110    xxxxx  */
   0x7F,  /* 01111111   xxxxxxx */
   0xC3,  /* 11000011  xx    xx */
   0x06,  /* 00000110       xx  */
   0x0C,  /* 00001100      xx   */
   0x18,  /* 00011000     xx    */
   0x30,  /* 00110000    xx     */
   0x60,  /* 01100000   xx      */
   0xFF,  /* 11111111  xxxxxxxx */
   0xFF,  /* 11111111  xxxxxxxx */

   0x3C,  /* 00111100    xxxx   */
   0x7E,  /* 01111110   xxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0x03,  /* 00000011        xx */
   0x0E,  /* 00001110      xxx  */
   0x0E,  /* 00001110      xxx  */
   0x03,  /* 00000011        xx */
   0xC3,  /* 11000011  xx    xx */
   0x7E,  /* 01111110   xxxxxx  */
   0x3C,  /* 00111100    xxxx   */

   0x06,  /* 00000110       xx  */
   0x0E,  /* 00001110      xxx  */
   0x1E,  /* 00011110     xxxx  */
   0x36,  /* 00110110    xx xx  */
   0x66,  /* 01100110   xx  xx  */
   0xC6,  /* 11000110  xx   xx  */
   0xFF,  /* 11111111  xxxxxxxx */
   0xFF,  /* 11111111  xxxxxxxx */
   0x06,  /* 00000110       xx  */
   0x06,  /* 00000110       xx  */

   0xFF,  /* 11111111  xxxxxxxx */
   0xFF,  /* 11111111  xxxxxxxx */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xFC,  /* 11111100  xxxxxx   */
   0xFE,  /* 11111110  xxxxxxx  */
   0x03,  /* 00000011        xx */
   0xC3,  /* 11000011  xx    xx */
   0x7E,  /* 01111110   xxxxxx  */
   0x3C,  /* 00111100    xxxx   */

   0x3E,  /* 00111110    xxxxx  */
   0x7C,  /* 01111100   xxxxx   */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xFC,  /* 11111100  xxxxxx   */
   0xFE,  /* 11111110  xxxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0x7E,  /* 01111110   xxxxxx  */
   0x3C,  /* 00111100    xxxx   */

   0xFF,  /* 11111111  xxxxxxxx */
   0xFF,  /* 11111111  xxxxxxxx */
   0x03,  /* 00000011        xx */
   0x06,  /* 00000110       xx  */
   0x0C,  /* 00001100      xx   */
   0x18,  /* 00011000     xx    */
   0x30,  /* 00110000    xx     */
   0x60,  /* 01100000   xx      */
   0x60,  /* 01100000   xx      */
   0x60,  /* 01100000   xx      */

   0x3C,  /* 00111100    xxxx   */
   0x7E,  /* 01111110   xxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0x7E,  /* 01111110   xxxxxx  */
   0x7E,  /* 01111110   xxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0x7E,  /* 01111110   xxxxxx  */
   0x3C,  /* 00111100    xxxx   */

   0x3C,  /* 00111100    xxxx   */
   0x7E,  /* 01111110   xxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0x7F,  /* 01111111   xxxxxxx */
   0x3F,  /* 00111111    xxxxxx */
   0x03,  /* 00000011        xx */
   0x03,  /* 00000011        xx */
   0x3E,  /* 00111110    xxxxx  */
   0x7C,  /* 01111100   xxxxx   */

   0x18,  /* 00011000     xx    */
   0x3C,  /* 00111100    xxxx   */
   0x66,  /* 01100110   xx  xx  */
   0xC3,  /* 11000011  xx    xx */
   0xFF,  /* 11111111  xxxxxxxx */
   0xFF,  /* 11111111  xxxxxxxx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */

   0xFC,  /* 11111100  xxxxxx   */
   0xFE,  /* 11111110  xxxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xFE,  /* 11111110  xxxxxxx  */
   0xFE,  /* 11111110  xxxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xFE,  /* 11111110  xxxxxxx  */
   0xFC,  /* 11111100  xxxxxx   */

   0x3C,  /* 00111100    xxxx   */
   0x7E,  /* 01111110   xxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xC3,  /* 11000011  xx    xx */
   0x7E,  /* 01111110   xxxxxx  */
   0x3C,  /* 00111100    xxxx   */

   0xFC,  /* 11111100  xxxxxx   */
   0xFE,  /* 11111110  xxxxxxx  */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xC3,  /* 11000011  xx    xx */
   0xFE,  /* 11111110  xxxxxxx  */
   0xFC,  /* 11111100  xxxxxx   */

   0xFF,  /* 11111111  xxxxxxxx */
   0xFF,  /* 11111111  xxxxxxxx */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xFE,  /* 11111110  xxxxxxx  */
   0xFE,  /* 11111110  xxxxxxx  */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xFF,  /* 11111111  xxxxxxxx */
   0xFF,  /* 11111111  xxxxxxxx */

   0xFF,  /* 11111111  xxxxxxxx */
   0xFF,  /* 11111111  xxxxxxxx */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xFC,  /* 11111100  xxxxxx   */
   0xFC,  /* 11111100  xxxxxx   */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xC0,  /* 11000000  xx       */
   0xC0   /* 11000000  xx       */
   };


BYTE keypad[NUM_KEYS] =  /*  Note: These values are read from PORT 60H when a key is pressed */
   {
   HEX_0_KEY,  /* 0x53  num-.     */
   HEX_1_KEY,  /* 0x47  num-7     */
   HEX_2_KEY,  /* 0x48  num-8     */
   HEX_3_KEY,  /* 0x49  num-9     */
   HEX_4_KEY,  /* 0x4B  num-4     */
   HEX_5_KEY,  /* 0x4C  num-5     */
   HEX_6_KEY,  /* 0x4D  num-6     */
   HEX_7_KEY,  /* 0x4F  num-1     */
   HEX_8_KEY,  /* 0x50  num-2     */
   HEX_9_KEY,  /* 0x51  num-3     */
   HEX_A_KEY,  /* 0x52  num-0     */
   HEX_B_KEY,  /* 0x1C  num-Enter */
   HEX_C_KEY,  /* 0x35  num-/     */
   HEX_D_KEY,  /* 0x37  num-*     */
   HEX_E_KEY,  /* 0x4A  num--     */
   HEX_F_KEY   /* 0x4E  num-+     */
   };

BYTE alt_keypad[NUM_KEYS] =  /*  Note: These values are read from PORT 60H when a key is pressed */
   {
   ALT_HEX_0_KEY,  /* 0x2D  x */
   ALT_HEX_1_KEY,  /* 0x02  1 */
   ALT_HEX_2_KEY,  /* 0x03  2 */
   ALT_HEX_3_KEY,  /* 0x04  3 */
   ALT_HEX_4_KEY,  /* 0x10  q */
   ALT_HEX_5_KEY,  /* 0x11  w */
   ALT_HEX_6_KEY,  /* 0x12  e */
   ALT_HEX_7_KEY,  /* 0x1E  a */
   ALT_HEX_8_KEY,  /* 0x1F  s */
   ALT_HEX_9_KEY,  /* 0x20  d */
   ALT_HEX_A_KEY,  /* 0x2C  z */
   ALT_HEX_B_KEY,  /* 0x2E  c */
   ALT_HEX_C_KEY,  /* 0x05  4 */
   ALT_HEX_D_KEY,  /* 0x13  r */
   ALT_HEX_E_KEY,  /* 0x21  f */
   ALT_HEX_F_KEY   /* 0x2F  v */
   };

BYTE chip8_keys[NUM_KEYS];
BYTE keyboard_int_flag;
BYTE key_state[128];  /* 0 if UP, otherwise pressed */
BYTE anykey();

BYTE use_alt_keypad;

WORD errorcode = ERROR_NONE;  /* need this as a global */

WORD peekw();

BYTE *chip8_error[] =
   {
   "Stack Overflow\n",
   "Stack Underflow\n",
   "Unknown Opcode\n",
   "PC out of range\n"
   };

/* CHIP-8 opcodes
 *
 *   00Cn - SCD nibble     SUPERCHIP instruction, scroll screen <nibble> lines DOWN
 *                         i.e. screen[i+nibble][] = screen[i][]
 *   00E0 - CLS            clear screen
 *   00EE - RET            return from subroutine
 *   00FB - SCR            SUPERCHIP instruction, scroll screen 4 pixels right
 *   00FC - SCL            SUPERCHIP instruction, scroll screen 4 pixels left
 *   00FD - EXIT           SUPERCHIP instruction, exit interpreter
 *   00FE - LOW            set LOWRES (128 x 32) graphics
 *   00FF - HIGH           set HIGHRES (256 x 64) graphics
 *   0nnn - SYS addr       <not implemented>
 *   1nnn - JP addr        JMP nnn
 *   2nnn - CALL addr      CALL nnn
 *   3xkk - SE Vx, byte    Skip next instr if Vx == KK
 *   4xkk - SNE Vx, byte   Skip next instr if Vx != KK
 *   5xy0 - SE Vx, Vy      Skip next instr if Vx == Vy
 *   6xkk - LD Vx, byte    Vx = KK
 *   7xkk - ADD Vx, byte   Vx += KK
 *   8xy0 - LD Vx, Vy      Vx = Vy
 *   8xy1 - OR Vx, Vy      Vx |= Vy
 *   8xy2 - AND Vx, Vy     Vx &= Vy
 *   8xy3 - XOR Vx, Vy     Vx ^= Vy
 *   8xy4 - ADD Vx, Vy     Vx += Vy, VF set to 1 if result > 0xff, else 0
 *   8xy5 - SUB Vx, Vy     Vx -= Vy, VF set to 1 if Vx >= Vy, else 0
 *   8xy6 - SHR Vx, 1      Vx >>= 1, VF set to 1 if carry, else 0
 *   8xy7 - SUBN Vx, Vy    Vx = Vy - Vx, VF set to 1 if Vy >= Vx, else 0
 *   8xyE - SHL Vx, 1      Vx <<= 1, VF set to 1 if carry, else 0
 *   9xy0 - SNE Vx, Vy     skip next instr if Vx != Vy
 *   Annn - LD I, addr     set I to NNN
 *   Bnnn - JP V0, addr    JMP to NNN + V0
 *   Cxkk - RND Vx, byte   Vx = <random> & KK
 *   Dxyn - DRW Vx, Vy, nibble  draw sprite at I at col=VX, row=VY for <nibble>
 *                              bytes.  VF set to 1 if screen pixels changed, else 0
 *                              If nibble == 0, draw 16-bit (highres) SPRITE
 *   Ex9E - SKP Vx         skip next instr if key stored in Vx is pressed
 *   ExA1 - SKNP Vx        skip next instr if key stored in Vx is not pressed
 *   Fx07 - LD Vx, DT      Vx = Delay_Timer
 *   Fx0A - LD Vx, K       Wait for key, then Vx = key
 *   Fx15 - LD DT, Vx      Delay_Timer = Vx
 *   Fx18 - LD ST, Vx      Sound_timer = Vx
 *   Fx1E - ADD I, Vx      I += Vx
 *   Fx29 - LD F, Vx       I = &system_sprite[Vx]
 *   Fx30 - LD HF, Vx      I = &highres_system_sprite[Vx]
 *   Fx33 - LD B, Vx       I[0] = BCD_HIGH(Vx), I[1] = BCD_MID(Vx), I[2] = BCD_LOW(Vx)
 *   Fx55 - LD [I], Vx     Save V0 .. Vx into memory at I .. I+x
 *   Fx65 - LD Vx, [I]     Load V0 .. Vx from memory at I .. I+x
 *   Fx75 - LD R, Vx       Save V0 .. Vx (x<8) to HP48 flags
 *   Fx85 - LD Vx, R       Load V0 .. Vx (x<8) from HP48 flags
 */
void instr_jmp(),
     instr_call(),
     instr_skipeq(),
     instr_skipne(),
     instr_skipregeq(),
     instr_skipregne(),
     instr_movkk(),
     instr_addkk(),
     instr_mov(),
     instr_or(),
     instr_and(),
     instr_xor(),
     instr_addreg(),
     instr_subreg(),
     instr_revsubreg(),
     instr_shr(),
     instr_shl(),
     instr_movi(),
     instr_jmpv0(),
     instr_rand(),
     instr_draw();

void instr_system();   /* opcode 0... collective processing */
void instr_math();     /* opcode 8... collective processing */
void instr_keys();     /* opcode E... collective processing */
void instr_memory();   /* opcode F... collective processing */  /* mostly memory... */

void (*primary_opcode[16])() =
   {
   instr_system,    /* 0... */
   instr_jmp,       /* 1... */
   instr_call,      /* 2... */
   instr_skipeq,    /* 3... */
   instr_skipne,    /* 4... */
   instr_skipregeq, /* 5... */
   instr_movkk,     /* 6... */
   instr_addkk,     /* 7... */
   instr_math,      /* 8... */
   instr_skipregne, /* 9... */
   instr_movi,      /* A... */
   instr_jmpv0,     /* B... */
   instr_rand,      /* C... */
   instr_draw,      /* D... */
   instr_keys,      /* E... */
   instr_memory     /* F... */  /* mostly memory... */
   };

void instr_bad_opcode();   /* unimmplemented math opcodes */
void (*math_opcode[16])() =
   {
   instr_mov,          /* ...0 */
   instr_or,           /* ...1 */
   instr_and,          /* ...2 */
   instr_xor,          /* ...3 */
   instr_addreg,       /* ...4 */
   instr_subreg,       /* ...5 */
   instr_shr,          /* ...6 */
   instr_revsubreg,    /* ...7 */
   instr_bad_opcode,   /* ...8 */
   instr_bad_opcode,   /* ...9 */
   instr_bad_opcode,   /* ...A */
   instr_bad_opcode,   /* ...B */
   instr_bad_opcode,   /* ...C */
   instr_bad_opcode,   /* ...D */
   instr_shl,          /* ...E */
   instr_bad_opcode    /* ...F */
   };

/* Note:  In DeSmet C, puts() does not automatically add a newline */
#ifdef COUNT_INSTRUCTIONS
DWORD start_time, end_time;
#endif

main(argc,argv)
   int argc;
   char *argv[];
   {
   int i, j;

   puts("CHIP8   Mar 8th 2020\n\n");
   for (i = 1; i < argc; i++)
      {
      if (argv[i][0] == '-')
         {
         switch(toupper(argv[i][1]))
            {
            case 'K':
               use_alt_keypad = TRUE;
               break;
            case 'F':
               execution_freerun = TRUE;
               break;
            case 'V':
               super_chip8_video = TRUE;
               screen_cols = GRAPHICS_COLS;
               screen_rows = GRAPHICS_ROWS;
               max_screen_cols = GRAPHICS_COLS - 1;
               max_screen_rows = GRAPHICS_ROWS - 1;
               break;
            case 'T':
               if (!isdigit(argv[i][2]))
                  help();
               sscanf(&argv[i][2],"%d",&j);
               if (j)
                  execution_ticks_per_instruction = j;
               break;
            case 'I':
               if (!isdigit(argv[i][2]))
                  help();
               sscanf(&argv[i][2],"%d",&j);
               if (j)
                  instructions_per_execution_tick = j;
               break;
            case 'S':
               if (!isdigit(argv[i][2]))
                  help();
               sscanf(&argv[i][2],"%d",&j);
               if ((j >= 100) && (j <= 1000))
                  sound_frequency = j;
               break;
            case 'L':
               if (!isdigit(argv[i][2]))
                  help();
               sscanf(&argv[i][2],"%d",&j);
               if ((j == PROGRAM_START) || (j == ETI660_PROGRAM_START))
                  program_load_addr = j;
               break;
            case 'W':
               wrap_sprites = TRUE;
               break;
            case 'M':
               use_vy_shift = TRUE;
               i_reg_is_incremented = TRUE;
               break;
            case 'X':
               use_vy_shift = TRUE;
               break;
            case 'Y':
               i_reg_is_incremented = TRUE;
               break;
            case 'Z':
               i_reg_sets_carry = TRUE;
               break;
            default:
               help();  /* this does not return */
               break;
            }
         for (j = i; j < argc-1; j++)
            strcpy(argv[j],argv[j+1]);
         argc--;
         i--;  /* remove this cmdline parameter */
         }
      }
   if (argc < 2)
      help();
   if ((i = open(argv[1], 0)) == -1)
      help();
   j = read(i,&memory[program_load_addr],MEM_SIZE - program_load_addr);
   close(i);
   if (j < MIN_PROGRAM_SIZE)
      {
      puts("Not a valid CHIP8 program.\n");
      exit(1);
      }
   j = peekw(SYSTEM_CLOCK_TICK_SEG, SYSTEM_CLOCK_TICK_OFS);
   i = 0;
   while (i < 18)
      {
      if (peekw(SYSTEM_CLOCK_TICK_SEG, SYSTEM_CLOCK_TICK_OFS) != j)
         {
         j = peekw(SYSTEM_CLOCK_TICK_SEG, SYSTEM_CLOCK_TICK_OFS);
         i++;
         }
      if (csts())   /* if key waiting */
         if (ci() == ESC_KEY)  /* get key */
            exit(0);
      }
   setup_chip8();
   run_chip8();
   reset_system();
   if (errorcode >= ERROR_STACK_OVERFLOW)
      {
      puts(chip8_error[errorcode - ERROR_STACK_OVERFLOW]);
      if (errorcode == ERROR_UNKNOWN_OPCODE)
         printf("opcode = %04x\n",opcode);
      }
#ifdef COUNT_INSTRUCTIONS
   if (end_time < start_time)
      end_time += 1572480L;  /* 86400 seconds/day * 18.2 ticks/second */
   end_time = (DWORD)(((float)(end_time - start_time) + 0.91) / 1.82);
                  /* integer execution time in seconds * 10 */
   printf("instruction count = %04x %04x %04x %04x in %.1f seconds\n",
               total_instruction_count[3],
               total_instruction_count[2],
               total_instruction_count[1],
               total_instruction_count[0],
               (float)end_time / 10.0  /* round 1/10ths of a second */
               );
#endif
   exit(errorcode);
   }

void instr_system()  /* 0... */
   {
/*   00Cn - SCD nibble     SUPERCHIP instruction, scroll screen <nibble> lines DOWN
 *                         i.e. screen[i+nibble][] = screen[i][]
 *   00E0 - CLS            clear screen
 *   00EE - RET            return from subroutine
 *   00FB - SCR            SUPERCHIP instruction, scroll screen 4 pixels right
 *   00FC - SCL            SUPERCHIP instruction, scroll screen 4 pixels left
 *   00FD - EXIT           SUPERCHIP instruction, exit interpreter
 *   00FE - LOW            SUPERCHIP instrction, set low graphics
 *   00FF - HIGH           SUPERCHIP instrctino, set high graphics
 */
   if ((opcode & 0xFF0) == 0x0C0)  /* SCD */
      {
      if (opcode_n = opcode_low & 0x0F)
         {
         scroll_screen_down();
         update_screen();
         }
      }
   else
      {
      switch (opcode & 0xFFF)
         {
         case 0xE0:   /* CLS */
            memfill(video_memory, sizeof(video_memory), BLACK);
            update_screen();
            break;
         case 0xEE:  /* RET */
            if (sp == STACK_START)
               errorcode = ERROR_STACK_UNDERFLOW;
            else
               {
                 /* here, we have
                 *   memory[sp-2] = return MSB
                 *   memory[sp-1] = return LSB
                 */
               *((BYTE *) &pc) = memory[++sp];
               *(((BYTE *) &pc)+1) = memory[++sp];
               }
            break;
         case 0xFB:  /* SCR */
            scroll_screen_right();
            update_screen();
            break;
         case 0xFC:  /* SCL */
            scroll_screen_left();
            update_screen();
            break;
         case 0xFD:
            errorcode = ERROR_EXIT;
            break;
         case 0xFE:  /* LOW */
            super_chip8_video = FALSE;
            screen_cols = LOWRES_GRAPHICS_COLS;
            screen_rows = LOWRES_GRAPHICS_ROWS;
            max_screen_cols = LOWRES_GRAPHICS_COLS - 1;
            max_screen_rows = LOWRES_GRAPHICS_ROWS - 1;
            memfill(video_memory, sizeof(video_memory), BLACK);
            setup_lowres_graphics();
            break;
         case 0xFF:  /* HIGH */
            super_chip8_video = TRUE;
            screen_cols = GRAPHICS_COLS;
            screen_rows = GRAPHICS_ROWS;
            max_screen_cols = GRAPHICS_COLS - 1;
            max_screen_rows = GRAPHICS_ROWS - 1;
            memfill(video_memory, sizeof(video_memory), BLACK);
            setup_highres_graphics();
            break;
         default:
            errorcode = ERROR_UNKNOWN_OPCODE;
            break;
         }
      }
   }

void instr_jmp()  /* 1... */
   {
   pc = opcode & ADDRESS_MASK;
   }

void instr_call()  /* 2... */
   {
   if (sp == (STACK_START - STACK_SIZE * 2))
      errorcode = ERROR_STACK_OVERFLOW;
   else
      {
      memory[sp--] = *(((BYTE *)&pc) + 1);
      memory[sp--] = *((BYTE *)&pc);
      pc = opcode & ADDRESS_MASK;
      }
   }

void instr_skipeq()  /* 3xkk - SE Vx, byte    Skip next instr if Vx == KK */
   {
   if (regs[X_REG_OPCODE_FIELD] == opcode_low)
      {
      pc += 2;  /* skip next instruction */
#ifdef CHECK_PC_LIMITS
      pc &= ADDRESS_MASK;  /* not sure if this is needed / wanted */
#endif
      }
   }

void instr_skipne()  /* 4xkk - SNE Vx, byte   Skip next instr if Vx != KK */
   {
   if (regs[X_REG_OPCODE_FIELD] != opcode_low)
      {
      pc += 2;  /* skip next instruction */
#ifdef CHECK_PC_LIMITS
      pc &= ADDRESS_MASK;  /* not sure if this is needed / wanted */
#endif
      }
   }

void instr_skipregeq()  /* 5xy0 - SE Vx, Vy      Skip next instr if Vx == Vy */
   {
   if (opcode_low & 0x0f)
      errorcode = ERROR_UNKNOWN_OPCODE;
   if (regs[X_REG_OPCODE_FIELD] == regs[Y_REG_OPCODE_FIELD])
      {
      pc += 2;  /* skip next instruction */
#ifdef CHECK_PC_LIMITS
      pc &= ADDRESS_MASK;  /* not sure if this is needed / wanted */
#endif
      }
   }

void instr_movkk()  /* 6xkk - LD Vx, byte    Vx = KK */
   {
   regs[X_REG_OPCODE_FIELD] = opcode_low;
   }

void instr_addkk()  /* 7xkk - ADD Vx, byte   Vx += KK */
   {
   regs[X_REG_OPCODE_FIELD] += opcode_low;
   }

void instr_mov()  /* 8xy0 - LD Vx, Vy      Vx = Vy */
   {
   regs[X_REG_OPCODE_FIELD] = regs[Y_REG_OPCODE_FIELD];
   }

void instr_or()  /* 8xy1 - OR Vx, Vy      Vx |= Vy */
   {
   regs[X_REG_OPCODE_FIELD] |= regs[Y_REG_OPCODE_FIELD];
   }

void instr_and()  /* 8xy2 - AND Vx, Vy     Vx &= Vy */
   {
   regs[X_REG_OPCODE_FIELD] &= regs[Y_REG_OPCODE_FIELD];
   }

void instr_xor()  /* 8xy3 - XOR Vx, Vy     Vx ^= Vy */
   {
   regs[X_REG_OPCODE_FIELD] ^= regs[Y_REG_OPCODE_FIELD];
   }

void instr_addreg()  /* 8xy4 - ADD Vx, Vy     Vx += Vy, VF set to 1 if result > 0xff, else 0 */
   {
   WORD temp;

   temp = (WORD)regs[opcode_x = X_REG_OPCODE_FIELD] + (WORD)regs[Y_REG_OPCODE_FIELD];
      /* DeSmet -- unless regs[] are explicitly typecast, this will be done
       *           as a BYTE addition then promoted to WORD, and we lose our
       *           carry information.
       */
   regs[opcode_x] = (BYTE)temp;
   regs[VF_REG] = (BYTE)(temp >> 8);
   }

void instr_subreg()  /* 8xy5 - SUB Vx, Vy     Vx -= Vy, VF set to 1 if Vx >= Vy, else 0 */
   {
   WORD temp;

   temp = (WORD)regs[opcode_x = X_REG_OPCODE_FIELD] - (WORD)regs[Y_REG_OPCODE_FIELD];
      /* DeSmet -- unless regs[] are explicitly typecast, this will be done
       *           as a BYTE subtraction then promoted to WORD, and we lose our
       *           carry information.
       */
   regs[opcode_x] = (BYTE)temp;
   regs[VF_REG] = (BYTE)(temp >> 8) + 1;
   }

void instr_shr()  /* 8xy6 - SHR Vx, 1      Vx >>= 1, VF set to 1 if carry, else 0 */
   {
   if (use_vy_shift)
      {
      regs[VF_REG] = regs[opcode_y = Y_REG_OPCODE_FIELD] & 1;
      regs[X_REG_OPCODE_FIELD] = regs[opcode_y] >> 1;
      }
   else
      {
      regs[VF_REG] = regs[opcode_x = X_REG_OPCODE_FIELD] & 1;
      regs[opcode_x] >>= 1;
      }
   }

void instr_revsubreg()  /* 8xy7 - SUBN Vx, Vy    Vx = Vy - Vx, VF set to 1 if Vy >= Vx, else 0 */
   {
   WORD temp;

   temp = (WORD)regs[Y_REG_OPCODE_FIELD] - (WORD)regs[opcode_x = X_REG_OPCODE_FIELD];
      /* DeSmet -- unless regs[] are explicitly typecast, this will be done
       *           as a BYTE subtraction then promoted to WORD, and we lose our
       *           carry information.
       */
   regs[opcode_x] = (BYTE)temp;
   regs[VF_REG] = (BYTE)(temp >> 8) + 1;
   }

void instr_shl()  /* 8xyE - SHL Vx, 1      Vx <<= 1, VF set to 1 if carry, else 0 */
   {
   if (use_vy_shift)
      {
      regs[VF_REG] = regs[opcode_y = Y_REG_OPCODE_FIELD] >> 7;
      regs[X_REG_OPCODE_FIELD] = regs[opcode_y] << 1;
      }
   else
      {
      regs[VF_REG] = regs[opcode_x = X_REG_OPCODE_FIELD] >> 7;
      regs[opcode_x] <<= 1;
      }
   }

void instr_bad_opcode()  /* 8..z */
   {
   errorcode = ERROR_UNKNOWN_OPCODE;
   }

void instr_math()  /* 8... */
   {
   (*math_opcode[opcode_low & 0x0f])();
   }

void instr_skipregne()  /* 9xy0 - SNE Vx, Vy     skip next instr if Vx != Vy */
   {
   if (opcode_low & 0x0f)
      errorcode = ERROR_UNKNOWN_OPCODE;
   if (regs[X_REG_OPCODE_FIELD] != regs[Y_REG_OPCODE_FIELD])
      {
      pc += 2;  /* skip next instruction */
#ifdef CHECK_PC_LIMITS
      pc &= ADDRESS_MASK;  /* not sure if this is needed / wanted */
#endif
      }
   }

void instr_movi()  /* Annn - LD I, addr     set I to NNN */
   {
   I_reg = opcode & ADDRESS_MASK;
   }

void instr_jmpv0()  /* Bnnn - JP V0, addr    JMP to NNN + V0 */
   {
   pc = (opcode & ADDRESS_MASK) + (WORD)regs[0];
   }

void instr_rand()  /* Cxkk - RND Vx, byte   Vx = <random> & KK */
   {
   regs[X_REG_OPCODE_FIELD] = rand() & opcode_low;
   }

void instr_draw()  /* Dxyn - DRW Vx, Vy, nibble  draw sprite at I at
                    * col=VX, row=VY for <nibble> bytes.
                    * VF set to 1 if screen pixels changed, else 0
                    */
   {
   BYTE spritedata;
   WORD sprite_16;
   WORD i, row, col, c;
   BYTE *p;

   if (!(opcode_n = opcode_low & 0x0F))
      opcode_n = 16;  /* 8 x 16 or 16 x 16 */
   regs[VF_REG] = WHITE;
   col = regs[X_REG_OPCODE_FIELD] & max_screen_cols;
   row = regs[opcode_low >> 4] & max_screen_rows;
   if (opcode_n + row > screen_rows)
      opcode_n = screen_rows - row;
   p = &video_memory[row * screen_cols];
   i = I_reg;
   if (!(opcode_low & 0x0F))  /* 16 x 16 sprite */
      {
      while (opcode_n)
         {
         *((BYTE *)&sprite_16 + 1) = memory[i++];
            /* 1st CHIP-8 memory is big-endian, to high byte of x86 var */
         *(BYTE *)&sprite_16 = memory[i++];
         if (wrap_sprites)
            {
            for (c = col; sprite_16; sprite_16 <<= 1, c = (c + 1) & max_screen_cols)
               {
               if (sprite_16 & 0x8000)
                  regs[VF_REG] &= (p[c] ^= WHITE);
               }
            }
         else
            {
            for (c = col; (sprite_16) && (c < screen_cols); sprite_16 <<= 1, c++)
               {
               if (sprite_16 & 0x8000)
                  regs[VF_REG] &= (p[c] ^= WHITE);
               }
            }
         opcode_n--;
         p += screen_cols;
         }
      }
   else
      {
      while (opcode_n)
         {
         if (wrap_sprites)
            {
            for (c = col, spritedata = memory[i++]; spritedata; spritedata <<= 1, c = (c + 1) & max_screen_cols)
               {
               if (spritedata & 0x80)
                  regs[VF_REG] &= (p[c] ^= WHITE);
               }
            }
         else
            {
            for (c = col, spritedata = memory[i++]; (spritedata) && (c < screen_cols); spritedata <<= 1, c++)
               {
               if (spritedata & 0x80)
                  regs[VF_REG] &= (p[c] ^= WHITE);
               }
            }
         opcode_n--;
         p += screen_cols;
         }
      }
   regs[VF_REG] = (regs[VF_REG] == WHITE) ? FALSE : TRUE;
   update_screen();
   }

void instr_keys()  /* Ex9E - SKP Vx         skip next instr if key stored in Vx is pressed
                    * ExA1 - SKNP Vx        skip next instr if key stored in Vx is not pressed
                    */
   {
   WORD temp;

   if (opcode_low == 0x9E)  /* skip if key Vx pressed */
      {
      if (((temp = regs[X_REG_OPCODE_FIELD]) < 0x10)  /* valid key number */
               && (key_state[chip8_keys[temp]]))
         {
         pc += 2;  /* skip next instr */
#ifdef CHECK_PC_LIMITS
         pc &= ADDRESS_MASK;  /* not sure if this is needed / wanted */
#endif
         }
      }
   else if (opcode_low == 0xA1)  /* skip if key Vx not pressed */
      {
      if (((temp = regs[X_REG_OPCODE_FIELD]) < 0x10)  /* valid key number */
               && (!key_state[chip8_keys[temp]]))
         {
         pc += 2;  /* skip next instr */
#ifdef CHECK_PC_LIMITS
         pc &= ADDRESS_MASK;  /* not sure if this is needed / wanted */
#endif
         }
      }
   else
      errorcode = ERROR_UNKNOWN_OPCODE;
   }

void instr_memory()  /* mostly memory... */
/*   Fx07 - LD Vx, DT      Vx = Delay_Timer
 *   Fx0A - LD Vx, K       Wait for key, then Vx = key
 *   Fx15 - LD DT, Vx      Delay_Timer = Vx
 *   Fx18 - LD ST, Vx      Sound_timer = Vx
 *   Fx1E - ADD I, Vx      I += Vx
 *   Fx29 - LD F, Vx       I = &system_sprite[Vx]
 *   Fx30 - LD HF, Vx      I = &highres_system_sprite[Vx]
 *   Fx33 - LD B, Vx       I[0] = BCD_HIGH(Vx), I[1] = BCD_MID(Vx), I[2] = BCD_LOW(Vx)
 *   Fx55 - LD [I], Vx     Save V0 .. Vx into memory at I .. I+x
 *   Fx65 - LD Vx, [I]     Load V0 .. Vx from memory at I .. I+x
 *   Fx75 - LD R, Vx       Save V0 .. Vx (x<8) to HP48 flags
 *   Fx85 - LD Vx, R       Load V0 .. Vx (x<8) from HP48 flags
 */
   {
   int temp;

   switch (opcode_low)
      {
      case 0x07:  /* Fx07 - LD Vx, DT      Vx = Delay_Timer */
         regs[X_REG_OPCODE_FIELD] = delay_timer;
         break;
      case 0x0A:  /* Fx0A - LD Vx, K       Wait for key, then Vx = key */
         if ((temp = anykey()) > 0x0F)
            pc -= 2;
               /* our "wait for key" is to keep executing this instruction.
                * We need to return to the main loop to do things like update
                * timers.
                */
         else
            regs[X_REG_OPCODE_FIELD] = temp;
         break;
      case 0x15:  /* Fx15 - LD DT, Vx      Delay_Timer = Vx */
         delay_timer = regs[X_REG_OPCODE_FIELD];
         break;
      case 0x18:  /* Fx18 - LD ST, Vx      Sound_timer = Vx */
         if (sound_timer = regs[X_REG_OPCODE_FIELD])
            sound_on();
         else
            sound_off();
         break;
      case 0x1e:  /* Fx1E - ADD I, Vx      I += Vx */
         I_reg += regs[X_REG_OPCODE_FIELD];
         if (i_reg_sets_carry)
            {
            if (I_reg >= MEM_SIZE)
               regs[VF_REG] = 1;
            else
               regs[VF_REG] = 0;
            }
         I_reg &= ADDRESS_MASK;
         break;
      case 0x29:  /* Fx29 - LD F, Vx       I = &system_sprite[Vx] */
         I_reg = HEX_SPRITES_LOCATION + (regs[X_REG_OPCODE_FIELD] & 0x0F) * HEX_SPRITES_ROWS;
         break;
      case 0x30:  /* Fx30 - LD HF, Vx      I = &highres_system_sprite[Vx] */
         I_reg = HEX_SPRITES_SUPER_LOCATION + (regs[X_REG_OPCODE_FIELD] & 0x0F) * HEX_SPRITES_SUPER_ROWS;
         break;
      case 0x33:  /* Fx33 - LD B, Vx       I[0] = BCD_HIGH(Vx), I[1] = BCD_MID(Vx), I[2] = BCD_LOW(Vx) */
         memory[I_reg] = (temp = regs[X_REG_OPCODE_FIELD]) / 100;
         memory[I_reg + 1] = (temp / 10) % 10;
         memory[I_reg + 2] = temp % 10;
         break;
      case 0x55:  /* Fx55 - LD [I], Vx     Save V0 .. Vx into memory at I .. I+x */
         for (opcode_x = X_REG_OPCODE_FIELD, temp = 0; temp <= opcode_x; temp++)
            memory[I_reg + temp] = regs[temp];
         if (i_reg_is_incremented)
            I_reg = (I_reg + opcode_x) & ADDRESS_MASK;
         break;
      case 0x65:  /* Fx65 - LD Vx, [I]     Load V0 .. Vx from memory at I .. I+x */
         for (opcode_x = X_REG_OPCODE_FIELD, temp = 0; temp <= opcode_x; temp++)
            regs[temp] = memory[I_reg + temp];
         if (i_reg_is_incremented)
            I_reg = (I_reg + opcode_x) & ADDRESS_MASK;
         break;
      case 0x75:  /* Fx75 - LD R, Vx       Save V0 .. Vx (x<8) to HP48 flags */
         if ((opcode_x = X_REG_OPCODE_FIELD) > 7)
            errorcode = ERROR_UNKNOWN_OPCODE;
         else
            {
            for (temp = 0; temp <= opcode_x; temp++)
               hp48regs[temp] = regs[temp];
            }
         break;
      case 0x85:  /* Fx85 - LD Vx, R       Load V0 .. Vx (x<8) from HP48 flags */
         if ((opcode_x = X_REG_OPCODE_FIELD) > 7)
            errorcode = ERROR_UNKNOWN_OPCODE;
         else
            {
            for (temp = 0; temp <= opcode_x; temp++)
               regs[temp] = hp48regs[temp];
            }
         break;
      default:
         errorcode = ERROR_UNKNOWN_OPCODE;
         break;
      }
   }

run_chip8()
   {
   static BYTE instruction_count;  /* put "local" in regular RAM for speed */

   while (errorcode == ERROR_NONE)  /* also use BREAK to get out */
      {
      if ((execution_freerun) || (execution_ticks_counter >= execution_ticks_per_instruction))
                /* >= in case of execution_ticks_per_instruction == 0 */
         {
#asm
   cli       ;do not disturb
#endasm
         execution_ticks_counter = 0;
#asm
   sti       ;ints are OK again
#endasm
         instruction_count = 0;  /* how many instructions this tick */
              /* x86 is little-endian, CHIP-8 memory is big-endian */
         while (TRUE)  /* use BREAK to get out */
            {
            if ((pc < program_load_addr) || (pc > MAX_PC))
               {
               errorcode = ERROR_PC_RANGE;
               break;
               }
            *((BYTE *)&opcode + 1) = opcode_high = memory[pc++];
               /* 1st CHIP-8 instr byte is high byte, to high-byte of x86 var */
            *(BYTE *)&opcode = opcode_low = memory[pc++];
#ifdef CHECK_PC_LIMITS
            pc &= ADDRESS_MASK;  /* not sure if this is needed / wanted */
#endif
            (*primary_opcode[opcode_high >> 4])();
#ifdef COUNT_INSTRUCTIONS
#asm
   add  word total_instruction_count_,1   ;do a 64-bit INC
   adc  word total_instruction_count_[2],0
   adc  word total_instruction_count_[4],0
   adc  word total_instruction_count_[6],0
#endasm
#endif
            if ((execution_freerun) || (++instruction_count >= instructions_per_execution_tick))
               break;
            }
         }
      if (sixty_hz_flag)
         {
         sixty_hz_flag = FALSE;
         if (sound_timer)
            {
            if (--sound_timer == 0)
               sound_off();
            }
         if (delay_timer)
            delay_timer--;
         if (key_state[ESC_KEY_SCANCODE])
            {
            errorcode = ERROR_EXIT;  /* not really an error, just convenient */
            }
         if (keyboard_int_flag)
            {
            keyboard_int_flag = FALSE;
            clear_keyboard_queue();
            }
              /* don't use these keypresses, just keep keyboard queue empty */
         }  /* sixty_hz_flag */
      }  /* while (errorcode == ERROR_NONE) */
   }

setup_highres_graphics()
   {
   WORD i;

   videomode(GRAPHICS_MODE);
   for (i = 0; i < 258; i++)
      {
      pixel(128,i,CYAN);
      pixel(129,i,CYAN);
      }
   for (i = 0; i < 129; i++)
      {
      pixel(i,256,CYAN);
      pixel(i,257,CYAN);
      }
   }

setup_lowres_graphics()
   {
   WORD i;

   videomode(GRAPHICS_MODE);
   for (i = 0; i < 320; i++)
      {
      pixel(160,i,CYAN);
      pixel(161,i,CYAN);
      }
   }

setup_chip8()
   {
   WORD i;

   for (i = 0; i < HEX_SPRITES_NUM_BYTES; i++)
      memory[HEX_SPRITES_LOCATION + i] = hex_sprites[i];
   for (i = 0; i < HEX_SPRITES_SUPER_NUM_BYTES; i++)
      memory[HEX_SPRITES_SUPER_LOCATION + i] = hex_sprites_super[i];

   if (use_alt_keypad)
      {
      for (i = 0; i < NUM_KEYS; i++)
         chip8_keys[i] = alt_keypad[i];
      }
   else
      {
      for (i = 0; i < NUM_KEYS; i++)
         chip8_keys[i] = keypad[i];
      }

   srand(peekw(SYSTEM_CLOCK_TICK_SEG, SYSTEM_CLOCK_TICK_OFS));

#ifdef COUNT_INSTRUCTIONS
   *((WORD *)&start_time + 1) = peekw(SYSTEM_CLOCK_TICK_SEG, SYSTEM_CLOCK_TICK_OFS + 2);
   *(WORD *)&start_time = peekw(SYSTEM_CLOCK_TICK_SEG, SYSTEM_CLOCK_TICK_OFS);
#endif

   pc = program_load_addr;

   setup_sound((WORD)(TIMER_CLOCK_BASE / sound_frequency));
   sound_off();

   takeover_keyboard_int();

   takeover_clock_tick(CHIP8_EXECUTION_TICKS_8253_DIVISOR);

   if (super_chip8_video)
      {
      setup_highres_graphics();
      }
   else
      {
      setup_lowres_graphics();
      }
   }

reset_system()
   {
   sound_off();

   reset_keyboard_int();

   reset_system_clock();

   videomode(TEXT_MODE);

#ifdef COUNT_INSTRUCTIONS
   *((WORD *)&end_time + 1) = peekw(SYSTEM_CLOCK_TICK_SEG, SYSTEM_CLOCK_TICK_OFS + 2);
   *(WORD *)&end_time = peekw(SYSTEM_CLOCK_TICK_SEG, SYSTEM_CLOCK_TICK_OFS);
#endif
   }

setup_sound(divisor)
   WORD divisor;
   {
#asm
   cli     ;DO NOT DISTURB!
   mov  al,0b6h             ;8253 control data:
                  ; 10.. .... = channel 2
                  ; ..11 .... = read/write LSB/MSB
                  ; .... 011. = generate square wave
                  ; .... ...0 = binary counting
   out   43h,al            ;set 8253 control port
   jmp   s_delay1          ;waste time
s_delay1:
   mov  ax,word [bp+4]     ;get divisor
   out   42h,al            ;set low byte of clock divisor
   jmp   s_delay2          ;waste time
s_delay2:
   mov   al,ah             ;get high byte
   out   42h,al            ;set high byte of divisor
   sti                     ;ints are OK again
#endasm
   }

sound_on()
   {
#asm
   in   al,061h    ;get speaker port
   or   al,3       ;turn on speaker
   out  061h,al    ;set speaker port
#endasm
   }

sound_off()
   {
#asm
   in   al,061h    ;get speaker port
   and  al,0fch    ;turn off speaker
   out  061h,al    ;set speaker port
#endasm
   }

takeover_keyboard_int()  /* includes INT 9 handler */
   {
#asm
   push ds                         ;save our DS
   mov  ax,ds                      ;get ds
   mov  word cs:dssave,ax          ;save DS for interrupt use
   xor  ax,ax                      ;get a 0
   mov  ds,ax                      ;address INT seg
   mov  si,word [24h]              ;get current int ofs
   mov  di,word [26h]              ;get current int seg
   mov  word cs:int9ofs,si         ;save ofs
   mov  word cs:int9seg,di         ;and seg
   mov  word [24h],offset int9here ;set new INT9 handler
   mov  word [26h],cs              ;and new INT9 seg
   pop  ds
   jmp  takeover_keyboard_int_end

dssave:  dw 0                      ;shared with INT 8 (timer) code
int9ofs: dw 0
int9seg: dw 0

int9here:
   pushf                           ;save flags
   push ax                         ;and regs
   push bx
   push ds
   mov  ds,word cs:dssave          ;get program's DS
   in   al,60h                     ;get keyboard port
   mov  bl,al                      ;copy key to addressing reg
   and  bx,7fh                     ;BX = index
   test al,80h                     ;highbit set (key release?)
   jz   kbd_int_key_down           ;no, skip this
   mov  byte key_state_[bx],0      ;this key not pressed
   jmp  kbd_int_continue           ;done for now
kbd_int_key_down:
   mov  byte keyboard_int_flag_,1  ;set flag: empty keyboard queue
   mov  byte key_state_[bx],1      ;this key is pressed
kbd_int_continue:
   pop  ds
   pop  bx
   pop  ax
   lcall word cs:int9ofs           ;do original INT 9
   iret                            ;and done

takeover_keyboard_int_end:
#endasm
   }

reset_keyboard_int()
   {
#asm
   push ds                         ;save our DS
   xor  ax,ax                      ;get a 0
   mov  ds,ax                      ;address INT seg
   mov  si,word cs:int9ofs         ;get ofs
   mov  di,word cs:int9seg         ;and seg
   mov  word [24h],si              ;save int ofs
   mov  word [26h],di              ;and int seg
   pop  ds
#endasm
   }

takeover_clock_tick(divisor)  /* includes INT 8 handler */
   WORD divisor;
   {
#asm
   cli     ;DO NOT DISTURB!
   push ds                         ;save our DS
   xor  ax,ax                      ;get a 0
   mov  ds,ax                      ;address INT seg
   mov  si,word [20h]              ;get current int ofs
   mov  di,word [22h]              ;get current int seg
   mov  word cs:int8ofs,si         ;save ofs
   mov  word cs:int8seg,di         ;and seg
   mov  word [20h],offset int8here ;set new INT8 handler
   mov  word [22h],cs              ;and new INT89 seg
   pop  ds
   mov  al,36h            ;8253 control data:
               ; 00.. .... = channel 0
               ; ..11 .... = read/write LSB/MSB
               ; .... 011. = generate square wave
               ; .... ...0 = binary counting
   out  43h,al            ;set 8253 control port
   jmp  delay1              ;waste time
delay1:
   mov  ax,word [bp+4]      ;get divisor
   out  40h,al            ;set low byte of clock divisor
   jmp  delay2              ;waste time
delay2:
   mov  al,ah            ;get high byte
   out  40h,al            ;set high byte of divisor
   sti                      ;ints are OK again
   jmp  takeover_tick_end   ;skip data storage

int8ofs: dw 0
int8seg: dw 0

;
;  Interrupt handler follows
;
int8here:
   push ds                  ;save ds
   push ax                  ;and ax
   mov  ds,word cs:dssave   ;get our ds
   inc  word execution_ticks_counter_  ;count this tick
   mov  byte execution_tick_flag_,1    ;set tick flag
   dec  byte sixty_hz_count_           ;update 60Hz counter
   jnz  int8_60hz_not_done             ;not expired, skip this
   mov  al,byte sixty_hz_divisor_      ;get counter value
   mov  byte sixty_hz_count_,al        ;reset counter
   mov  byte sixty_hz_flag_,1          ;set flag for interpreter
int8_60hz_not_done:
   dec  byte system_clock_count_       ;update 18.2Hz counter
   jnz  int8_system_clock_not_done     ;not expired, skip this
   mov  al,byte system_clock_divisor_  ;get counter value
   mov  byte system_clock_count_,al    ;reset counter
   pushf                               ;save flags (simulate interrupt)
   lcall word cs:int8ofs               ;far CALL to original INT 8, DeSmet style
int8_system_clock_not_done:
   mov  al,20h              ;EOI signal
   out  20h,al              ;send EOI to 8259
   pop  ax                  ;restore ax
   pop  ds                  ;and ds
   iret                     ;and done
;
;  End of Interrupt Handler
;

takeover_tick_end:
#endasm
   }

reset_system_clock()
   {
#asm
   push ds                         ;save our DS
   xor  ax,ax                      ;get a 0
   mov  ds,ax                      ;address INT seg
   mov  si,word cs:int8ofs         ;get ofs
   mov  di,word cs:int8seg         ;and seg
   mov  word [20h],si              ;save int ofs
   mov  word [22h],di              ;and int seg
   pop  ds
   mov  al,36h                     ;8253 control data:
               ; 00.. .... = channel 0
               ; ..11 .... = read/write LSB/MSB
               ; .... 011. = generate square wave
               ; .... ...0 = binary counting
   out  43h,al                     ;set 8253 control port
   jmp  delay10                    ;waste time
delay10:
   xor  ax,ax                      ;get divisor of 65536
   out  40h,al                     ;set low byte of clock divisor
   jmp  delay11                    ;waste time
delay11:
   out  40h,al                     ;set high byte of divisor
#endasm
   }

videomode(mode)
   int mode;
   {
#asm
   mov  ah,0               ;fn = set video mode
   mov  al,byte [bp+4]     ;get new mode
   int  10h                ;call BIOS
#endasm
   }

pokew(seg,ofs,value)
   unsigned int seg,ofs,value;
   {
#asm
   mov  es,word [bp+4]  ;get seg
   mov  bx,word [bp+6]  ;get ofs
   mov  ax,word [bp+8]  ;get value
   mov  word es:[bx],ax ;poke the word
   push ds              ;current seg
   pop  es              ;to es
#endasm
   }

unsigned int peekw(seg,ofs)
   unsigned int seg,ofs;
   {
#asm
   mov  es,word [bp+4]  ;get seg
   mov  bx,word [bp+6]  ;get ofs
   mov  ax,word es:[bx] ;get value
   push ds              ;current seg
   pop  es              ;to es
#endasm
   }

memmove(src, dest, size)
   BYTE *src, *dest;
   WORD size;
   {
#asm
   mov  si,word [bp+4]            ;get source addr
   mov  di,word [bp+6]            ;get dest addr
   mov  cx,word [bp+8]            ;get bytes to move
   cld                            ;auto-inc string primitives
   shr  cx,1                      ;to words, CY flag is "odd byte count"
   db   0f3h                      ;REPZ, DeSmet doesn't do it right
   movsw                          ;move memory
   jnc memmove_done               ;if no "odd byte", skip this
   movsb                          ;move the last byte
memmove_done:
#endasm
   }

memfill(dest, size, value)
   BYTE *dest;
   WORD size;
   BYTE value;
   {
#asm
   mov  di,word [bp+4]            ;get dest addr
   mov  cx,word [bp+6]            ;get byte count
   mov  al,byte [bp+8]            ;get value
   mov  ah,al                     ;make it a WORD
   cld                            ;auto-inc string primitives
   shr  cx,1                      ;byte count to WORDS, CY flag is "odd byte count"
   db   0f3h                      ;REPZ, DeSmet doesn't do it right
   stosw                          ;fill memory
   jnc  memfill_done              ;if no "odd byte", skip this
   stosb                          ;fill the last byte
memfill_done:
#endasm
   }

clear_keyboard_queue()
   {
#asm
clear_keyboard_queue_loop:
   mov  ax,1100h     ;fn = check extended keypress
   int  16h          ;call BIOS
   jz   clear_keyboard_queue_done  ;nope, we're done
   mov  ax,1000h     ;fn = get extended keypress
   int  16h          ;call BIOS
   jmp  clear_keyboard_queue_loop  ;more keys, keep going
clear_keyboard_queue_done:
#endasm
   }

BYTE anykey()
   {
   WORD i;

   for (i = 0; i < NUM_KEYS; i++)
      if (key_state[chip8_keys[i]])
         return(i);
   return(0xff);
   }

scroll_screen_down()
      /* Note: We know the screen always has an even number of ROWS and
       *       COLUMNS, so we can do WORD moves without worry about
       *       an odd number of bytes.
       */
   {
   if (super_chip8_video)  /* 128 columns, 64 rows */
      {
#asm
   mov  di,offset video_memory_[8190]    ;ES:DI pointer to end of video memory
   mov  si,di            ;and a copy to SI
   xor  ax,ax            ;clear ax
   mov  ah,byte opcode_n_ ;ax = lines to move * 256
   shr  ax,1             ;/2 for lines to move * 128 (128 == bytes per line)
   mov  cx,8192          ;video memory size
   sub  cx,ax            ;bytes to move
   shr  cx,1             ;words to move
   sub  si,ax            ;DS:SI = end of video memory to move
   std                   ;auto-dec string primitives
   db   0f3h             ;REP
   movsw                 ;move video memory down
   mov  cx,ax            ;number of bytes that were moved
   shr  cx,1             ;number of words moved == number of words to fill
   mov  al,byte black_   ;get fill color
   mov  ah,al            ;as a WORD
   db   0f3h             ;REP
   stosw                 ;erase lines that were moved down
#endasm
      }
   else  /* 64 columns, 32 rows */
      {
      if (opcode_n == 1)
         return;
          /* we must move a full line (no 1/2 lines).  If we are only
           * scolling 1 line, 1/2 of that is a "null move", do nothing.
           *
           * I am not going to do like some CHIP-8 interpreters and move
           * 1/2 a line (i.e. start in the middle of a line)
           */
#asm
   mov  di,offset video_memory_[2046]    ;ES:DI pointer to end of video memory
   mov  si,di              ;and a copy to SI
   xor  ax,ax              ;clear ax
   mov  ah,byte opcode_n_  ;ax = lines to move * 256
   shr  ax,1               ;/2 = lines to move * 128
   shr  ax,1               ;/2 = lines to move * 64
   shr  ax,1               ;/2 = lines to move * 32
                           ;== (lines to move / 2) * bytes per line
                           ;1/2 the sceen resolution, 1/2 the lines
   mov  cx,2048            ;video memory size
   sub  cx,ax              ;bytes to move
   shr  cx,1               ;words to move
   sub  si,ax              ;DS:SI = end of video memory to move
   std                     ;auto-dec string primitives
   db   0f3h               ;REP
   movsw                   ;move video memory down
   mov  cx,ax              ;number of bytes that were moved
   shr  cx,1               ;number of words moved == number of words to fill
   mov  al,byte black_     ;get fill color
   mov  ah,al              ;as a WORD
   db   0f3h               ;REP
   stosw                   ;erase lines that were moved down
#endasm
      }
   }

update_screen()
      /* Note: We know the screen always has an even number of ROWS and
       *       COLUMNS, so we can do WORD moves without worrying about
       *       an odd number of bytes.
       */
   {
   if (super_chip8_video)  /* 128 columns, 64 rows */
      {
    /* Since the screen is 320 x 240, and 128 * 3 == 384 > 320,
     * a "pixel" will be a 2 x 2 block.
     */
#asm
   push es                           ;save current es
   mov  bx,offset video_memory_      ;bx = video data
   mov  es,word video_seg_           ;get target video seg
   xor  di,di                        ;es:di = video memory
   mov  dl,64                        ;video lines
   cld                               ;auto-inc string primitives
update_screen_super_1:
   mov  dh,2                         ;pixels are 2 x 2, line count
update_screen_super_2:
   mov  si,bx                        ;ds:si = video data
   mov  cx,128                       ;bytes per line
update_screen_super_3:
   lodsb                             ;get video data
   stosb                             ;target pixel is 2 x 2
   stosb
   loop  update_screen_super_3       ;do entire row
   add   di,320-256                  ;point to next line in video seg
   dec   dh                          ;count this pixel line
   jnz   update_screen_super_2       ;do next line of 2 x 2 pixel
   add   bx,128                      ;offset of next line of video data
   dec   dl                          ;count this line
   jnz   update_screen_super_1       ;do all lines
   pop   es                          ;restore old es
#endasm
      }
   else  /* 64 columns, 32 rows */
      {
   /* Since the screen is 320 x 240, and 64 * 5 == 320 and 32 * 5 == 160,
    * a "pixel" will be a 5 x 5 block.
    */
#asm
   push es                           ;save current es
   mov  bx,offset video_memory_      ;bx = video data
   mov  es,word video_seg_           ;get target video seg
   xor  di,di                        ;es:di = video memory
   mov  dl,32                        ;video lines
   cld                               ;auto-inc string primitives
update_screen_chip8_1:
   mov  dh,5                         ;pixels are 5 x 5, line count
update_screen_chip8_2:
   mov  si,bx                        ;ds:si = video data
   mov  cx,64                        ;bytes per line
update_screen_chip8_3:
   lodsb                             ;get video data
   mov  ah,al                        ;as a WORD
   stosw                             ;target pixel is 5 x 5
   stosw
   stosb
   loop  update_screen_chip8_3       ;do entire row
                                     ;Note:  Since 64 * 5 == 320, di already
                                     ;       points to next line in video seg
   dec   dh                          ;count this pixel line
   jnz   update_screen_chip8_2       ;do next line of 5 x 5 pixel
   add   bx,64                       ;point to next line in video memory
   dec   dl                          ;count this line
   jnz   update_screen_chip8_1       ;do all lines
   pop   es                          ;restore old es
#endasm
      }
   }

scroll_screen_right()
   {
   if (super_chip8_video)  /* 128 columns, 64 rows */
      {
    /* for all rows,
     *  c0..c3=BLACK, c4..c127 = old_c0..old_c123
     */
#asm
   mov  bx,offset video_memory_[127]  ;destination (row 0)
   mov  dl,64                         ;rows to process
   std                                ;auto-dec string primitives
   mov  al,byte black_                ;get fill color
   mov  ah,al                         ;as a WORD
scroll_right_super:
   mov  di,bx                         ;es:di = destination
   lea  si,[di-4]                     ;ds:si = source
   mov  cx,62                         ;words to move ((128-4)/2)
   db   0f3h                          ;REPZ
   movsw                              ;move video data
   stosw                              ;fill first 4 bytes
   stosw
   add  bx,128                        ;offset in next rown
   dec  dl                            ;count this row
   jnz  scroll_right_super            ;do all rows
#endasm
      }
   else  /* 64 columns, 32 rows */
      {
    /* for all rows,
     *  c0..c1=BLACK, c2..c63 = old_c0..old_c61
     */
#asm
   mov  bx,offset video_memory_[63]   ;destination (row 0)
   mov  dl,32                         ;rows to process
   std                                ;auto-dec string primitives
   mov  al,byte black_                ;get fill color
   mov  ah,al                         ;as a WORD
scroll_right_chip8:
   mov  di,bx                         ;es:di = destination
   lea  si,[di-2]                     ;ds:si = source
   mov  cx,31                         ;words to move ((64-2)/2)
   db   0f3h                          ;REPZ
   movsw                              ;move video data
   stosw                              ;fill first 2 bytes
   add  bx,64                         ;offset in next row
   dec  dl                            ;count this row
   jnz  scroll_right_chip8            ;do all rows
#endasm
      }
   }

scroll_screen_left()
   {
   if (super_chip8_video)  /* 128 columns, 64 rows */
      {
    /* for all rows,
     *  c0 .. c123 --> old_c4..old_c123, c124..c127=BLACK
     */
#asm
   mov  bx,offset video_memory_       ;destination (row 0)
   mov  dl,64                         ;rows to process
   cld                                ;auto-inc string primitives
   mov  al,byte black_                ;get fill color
   mov  ah,al                         ;as a WORD
scroll_left_super:
   mov  di,bx                         ;es:di = destination
   lea  si,[di+4]                     ;ds:si = source
   mov  cx,62                         ;words to move ((128-4)/2)
   db   0f3h                          ;REPZ
   movsw                              ;move video data
   stosw                              ;fill last 4 bytes
   stosw
   add  bx,128                        ;offset in next row
   dec  dl                            ;count this row
   jnz  scroll_left_super             ;do all rows
#endasm
      }
   else  /* 64 columns, 32 rows */
      {
    /* for all rows,
     *  c0 .. c61 --> old_c2..old_c63, c62..c63=BLACK
     */
#asm
   mov  bx,offset video_memory_       ;destination (row 0)
   mov  dx,32                         ;rows to process
   cld                                ;auto-inc string primitives
   mov  al,byte black_                ;get fill color
   mov  ah,al                         ;as a WORD
scroll_left_chip8:
   mov  di,bx                         ;es:di = destination
   lea  si,[di+2]                     ;ds:si = source
   mov  cx,31                         ;words to move ((128-4)/2)
   db   0f3h                          ;REPZ
   movsw                              ;move video data
   stosw                              ;fill last 2 bytes
   add  bx,64                         ;offset in next row
   dec  dl                            ;count this row
   jnz  scroll_left_chip8             ;do all rows
#endasm
      }
   }

pixel(r,c,color)
   WORD r,c,color;
   {
#asm
   mov  ah,0ch
   mov  al,byte [bp+8]
   mov  bx,0
   mov  cx,word [bp+6]
   mov  dx,word [bp+4]
   int  10h
#endasm
   }

help()
   {
   puts("CHIP8 [options] chip8_program\n");
   puts("Options:\n");
   puts("   -K  : Use Alternate keypad (1..4/q..r/a..f/z..v) instead of PC keypad.\n");
   puts("   -F  : execution Freerun (run code as fast as possible)\n");
   puts("   -V  : Default High-res video (128 x 64) instead of standard (64 x 32)\n");
   puts("   -Tn : Execution ticks (2040 ticks/second) to wait before executing\n");
   puts("         CHIP8 instructions.  Default is 1.\n");
   puts("   -In : CHIP8 instructions to execute every time execution tick timer\n");
   puts("         expires.  Default is 1.\n");
   puts("   -Sn : Sound frequency.  Default is 330Hz.  Range is 100 to 1000.\n");
   puts("   -Ln : Load address for program.  Valid values are 512 and 1536.\n");
   puts("         Default value is 512 (0x0200).\n");
   puts("   -W  : Wrap sprites that go beyond the right edge of the screen.\n");
   puts("         Default is to truncate sprites at the right edge of the screen.\n");
   puts("   -M  : Use Mikolay's instruction behaviors (equivalent to using both\n");
   puts("         -X and -Y options).\n");
   puts("   -X  : SHR Vx,Vy and SHL Vx,Vy -- shift Vy register and store it in Vx\n");
   puts("         Default is that Vx is shifted.\n");
   puts("   -Y  : LD [I],Vx and LD Vx,[I] instructions DO update the I register.\n");
   puts("         Default is that they DO NOT update the I register.\n");
   puts("   -Z  : ADD I,Vx DOES sets carry in VF\n");
   puts("         Default is that carry is NOT set in VF.\n");
   exit(10);
   }
