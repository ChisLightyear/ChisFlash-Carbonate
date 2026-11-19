#include "main.h"
#include "usb.h"
#include "uart.h"
#include "cart_adapter.h"
#include "flashgbx.h"
#include "gba_save.h"
#include "fgx_state.h"
#include "lk_special_api.h"
#include <string.h>

#ifdef FLASHGBX_COMPAT
#include "crc32_table.h"
/* 协议数值为大端，ROM 数据保持原始字节顺序。 */
extern uint16_t cmdBuf_i_wr, cmdBuf_i_rd;
extern uint8_t xdata cmdBuf[5500], responBuf[EP1IN_SIZE];
void endpointClear(void) FGX_STACK;
#define FGX_LIMIT 4096
#define FGX_READ_LIMIT 8192
FgxState xdata fgx_state;
#define state fgx_state
static FgxState xdata saved_state;
static bit active;
static uint8_t pending, pending_type;
static volatile uint8_t session_id;
volatile bit flashgbx_bus_cancelled;
volatile bit flashgbx_stream_polling;
static bit stream_poll_enabled=1;
static bit stream_read_enabled=1;
uint8_t xdata flashgbx_crc_fast_mode=2;
uint8_t xdata flashgbx_program_fast_mode=1;
bit flashgbx_rx_burst=1;
static bit dmg_buffer_fast=1;
static uint8_t stream_saved_in_mask;
static uint8_t transaction_id;
static bit transport_fault;
static volatile uint16_t idle_ticks;
static uint16_t memory3d_offset;
static bit memory3d_active;
static uint8_t xdata reply[64], bus_word[2];
static uint8_t code probe_reply[] = "\x01" "ChisFlash FW C15 RC39-JRBASE\r\n";
#define ADDRESS state.wide[0]
#define TRANSFER state.word[0]
#define BUFFER state.word[1]
#define MODE state.byte[0]
#define voltage state.selected_voltage
static void apply_aux_pin(void);
static void set_pullups(uint8_t flags);
static void stream_poll_end(void) {
    uint8_t saved_ie;
    if(!flashgbx_stream_polling) return;
    saved_ie=IE2;IE2&=~EUSB;
    usb_write_reg(INTRIN1E,stream_saved_in_mask);
    flashgbx_stream_polling=0;
    IE2=saved_ie;
}
static void stream_poll_begin(void) {
    uint8_t saved_ie;
    if(!stream_poll_enabled) return;
    saved_ie=IE2;IE2&=~EUSB;
    stream_saved_in_mask=usb_read_reg(INTRIN1E);
    flashgbx_stream_polling=1;
    usb_write_reg(INTRIN1E,stream_saved_in_mask&~EP1INIE);
    IE2=saved_ie;
}
static void select_power(uint8_t mode) {
    board_setPower(mode);
    /* 覆盖慢启动存储器；按当前 MCU 时钟保留充足的上电稳定窗口。 */
    if(mode && board_getPower()) { set_pullups(state.byte[14]); lk_delay_us(50000); lk_delay_us(50000); lk_delay_us(50000); }
}
bit flashgbx_cancelled(void) { return transaction_id != session_id; }
void flashgbx_tick(void) { if(idle_ticks<65535) ++idle_ticks; }
static void idle_poweroff(void) {
    uint16_t ticks;
    uint8_t saved_ea=EA;
    EA=0; ticks=idle_ticks; EA=saved_ea;
    if(state.byte[15] && state.auto_ticks && ticks>=state.auto_ticks && board_getPower()) {
        CART_CS1=1; CART_CS2=1; CART_RD=1; CART_WR=1;
        cart_setDirection_ad(0); cart_setDirection_a(0); board_setPower(0);
        apply_aux_pin();
    }
}

static uint32_t be32(uint16_t p) {
    return ((uint32_t)cmdBuf[p]<<24)|((uint32_t)cmdBuf[p+1]<<16)|((uint32_t)cmdBuf[p+2]<<8)|cmdBuf[p+3];
}
static uint16_t be16(uint16_t p) { return ((uint16_t)cmdBuf[p]<<8)|cmdBuf[p+1]; }
/* 配置中的 AMD 命令已按卡槽位序编码；只转换固件临时生成的控制字段，不转换 ROM 数据。 */
static uint8_t amd_wire_byte(uint8_t value,uint8_t unlock) {
    if((unlock&3)==1 && ((value^(value>>1))&1)) value^=3;
    if((unlock&0xc0)==0x40 && ((value^(value>>1))&0x40)) value^=0xc0;
    return value;
}
static bit amd_commands_valid(void) {
    uint8_t unlock=(uint8_t)be16(8);
    return unlock==amd_wire_byte(0xaa,unlock) &&
        (uint8_t)be16(14)==amd_wire_byte(0x55,unlock) &&
        (uint8_t)be16(20)==amd_wire_byte(cmdBuf[2]==1?0xa0:0x25,unlock) &&
        (cmdBuf[2]!=2 || (uint8_t)be16(38)==amd_wire_byte(0x29,unlock));
}
static void put32(uint32_t v) { reply[0]=v>>24; reply[1]=v>>16; reply[2]=v>>8; reply[3]=v; }

#ifdef __SDCC
/* 调用方已屏蔽 USB ISR；DPTR 连续读取 XRAM，保留 USB 外设忙检查。
   显式保留寄存器，零长度仍由调用方提交 ZLP。 */
static void usb_fifo_write_xram(const uint8_t xdata *data,uint8_t count) __naked {
    (void)data; (void)count;
    __asm
        push acc
        push psw
        push dpl
        push dph
        push ar0
        push ar1
        push ar7
        mov r0,dpl
        mov r1,dph
        mov dptr,#_usb_fifo_write_xram_PARM_2
        movx a,@dptr
        mov r7,a
        mov dpl,r0
        mov dph,r1
        jz 00982$
    00981$:
        mov a,_USBADR
        jb acc.7,00981$
        movx a,@dptr
        mov _USBADR,#0x21
        mov _USBDAT,a
        inc dptr
        djnz r7,00981$
    00982$:
        pop ar7
        pop ar1
        pop ar0
        pop dph
        pop dpl
        pop psw
        pop acc
        ret
    __endasm;
}
#endif
/* 回复期间保持 OUT 背压；处理后才移除输入，避免下一条命令被清空。 */
/* 专用低页缓冲区：每组四字节减少循环开销，仍逐字节检查 USB 外设忙。 */
#ifdef __SDCC
static void usb_fifo_write_low64(void) __naked {
    __asm
        push acc
        push psw
        push ar0
        push ar7
        mov r0,#_fgx_read_page
        mov r7,#16
    00961$:
        mov a,_USBADR
        jb acc.7,00961$
        movx a,@r0
        mov _USBADR,#0x21
        mov _USBDAT,a
        inc r0
    00962$:
        mov a,_USBADR
        jb acc.7,00962$
        movx a,@r0
        mov _USBADR,#0x21
        mov _USBDAT,a
        inc r0
    00963$:
        mov a,_USBADR
        jb acc.7,00963$
        movx a,@r0
        mov _USBADR,#0x21
        mov _USBDAT,a
        inc r0
    00964$:
        mov a,_USBADR
        jb acc.7,00964$
        movx a,@r0
        mov _USBADR,#0x21
        mov _USBDAT,a
        inc r0
        djnz r7,00961$
        pop ar7
        pop ar0
        pop psw
        pop acc
        ret
    __endasm;
}
#endif
static bit send_packet(const uint8_t xdata *data, uint8_t size) {
    uint16_t guard=60000;
    uint8_t saved_ie;
    while(ep1Busy && guard && !flashgbx_cancelled()) {
        if(flashgbx_stream_polling) {
            saved_ie=IE2;IE2&=~EUSB;
            usb_write_reg(INDEX,1);
            if(!(usb_read_reg(INCSR1)&INIPRDY)) ep1Busy=0;
            IE2=saved_ie;
        }
        --guard;
    }
    if(flashgbx_cancelled()) return 0;
    if(ep1Busy) { transport_fault=1; return 0; }
    saved_ie=IE2; IE2&=~EUSB;
    usb_write_reg(INDEX,1);
#ifdef __SDCC
    if(size==64 && data==fgx_read_page) usb_fifo_write_low64();
    else if(size>8) usb_fifo_write_xram(data,size);
    else
#endif
    while(size--) {
        while(USBADR&0x80) ;
        USBADR=FIFO1;
        USBDAT=*data++;
    }
    ep1Busy=1;
    usb_write_reg(INCSR1,INIPRDY);
    IE2=saved_ie;
    return 1;
}
static void ack(uint8_t v) { reply[0]=v; send_packet(reply,1); }
static void state_defaults(void) {
    memset(&state,0,sizeof(state));
    state.byte[14]=1; state.word[5]=0x80; state.word[6]=0x80;
    voltage=board_getPower(); if(!voltage) voltage=1;
}
bit fgx_send_word(uint16_t value) FGX_STACK {
    reply[0]=value>>8; reply[1]=value;
    return send_packet(reply,2);
}
static void consume(uint16_t count) {
    uint16_t i;
    uint8_t saved_ie=IE2;
    uint8_t saved_ea;
    IE2&=~EUSB;
    if(flashgbx_cancelled()) { state_defaults(); IE2=saved_ie; return; }
    if(transport_fault) {
        cmdBuf_i_wr=0; cmdBuf_i_rd=0; pending=0;
        IE2=saved_ie; return;
    }
    if(count>cmdBuf_i_wr) count=cmdBuf_i_wr;
    for(i=count;i<cmdBuf_i_wr;i++) cmdBuf[i-count]=cmdBuf[i];
    cmdBuf_i_wr-=count; cmdBuf_i_rd=0;
    /* 命令执行期间不计作空闲；原子重置，避免 Timer4 打断 16 位赋值。 */
    saved_ea=EA; EA=0; idle_ticks=0; EA=saved_ea;
    endpointClear();
    IE2=saved_ie;
}
void flashgbx_reset(void) {
    LED_ACT=1;
    flashgbx_bus_cancelled=1;
    cart_gbc_hold_writes=0;
    stream_poll_end();
    ++session_id;
    CART_CS1=1; CART_CS2=1; CART_RD=1; CART_WR=1;
    CART_IRQ=1; SetP4nHighZInputMode(PIN_5);
    active=0; pending=0; transport_fault=0; idle_ticks=0; memory3d_active=0;
    state_defaults();
    cmdBuf_i_wr=0; cmdBuf_i_rd=0; ep1Busy=0;
}
bit flashgbx_is_active(void) { return active; }
void flashgbx_mark_receive(void) {
    idle_ticks=0;
    /* FlashGBX 探测后会重开串口并直接 AD 查询，不会再次发送 A1。 */
    if(cmdBuf_i_wr && (cmdBuf[0]>=0xa0 || cmdBuf[0]==0x55)) active=1;
}
static uint16_t expected(void) {
    if(pending==0xaf) return sizeof(state);
    if(pending) return TRANSFER;
    if(!cmdBuf_i_wr) return 1;
    switch(cmdBuf[0]) {
    case 0x55: case 0x4c: case 0xfe: case 0xc5: case 0xc6: case 0xc7: return 2;
    case 0xa6: return 10;
    case 0xad: case 0xb2: case 0xd1: case 0xf5: return 6;
    case 0xc2: case 0xd2: return 7;
    case 0xa9: case 0xd5: return 5;
    case 0xa7: return 40;
    case 0xb8: return cmdBuf_i_wr<2?2:2+(uint16_t)cmdBuf[1]*5;
    case 0xd4: return cmdBuf_i_wr<3?3:3+(uint16_t)cmdBuf[2]*6;
    default: return 1;
    }
}
bit flashgbx_packet_complete(void) { return cmdBuf_i_wr && cmdBuf_i_wr>=expected(); }
static void query_info(void) {
    uint8_t i;
    reply[0]=8; reply[1]='C'; reply[2]=0; reply[3]=15;
    reply[4]=2; /* Joey 后端的软件供电兼容类型，设备名称仍为 ChisFlash。 */
    reply[5]=0x6a; reply[6]=0x20; reply[7]=0x68; reply[8]=0x1e; reply[9]=9;
    for(i=0;i<9;i++) reply[10+i]="ChisFlash"[i];
    reply[19]=1; reply[20]=0;
    send_packet(reply,21);
}
static uint32_t get_variable(uint8_t size,uint32_t key) {
    if(size==1 && key==18) return stream_poll_enabled;
    if(size==1 && key==19) return stream_read_enabled;
    if(size==1 && key==20) return flashgbx_crc_fast_mode;
    if(size==1 && key==21) return flashgbx_program_fast_mode;
    if(size==1 && key==22) return flashgbx_rx_burst;
    if(size==1 && key==24) return dmg_buffer_fast;
    if(size==1 && key==25) return flashgbx_dmg_pipeline;
    if(size==1 && key==26) return flashgbx_gbc_batch_fast;
    if(size==1 && key==27) return flashgbx_gbc_fast_timing;
    if(size==4 && key<2) return state.wide[(uint8_t)key];
    if(size==2 && key<7) return state.word[(uint8_t)key];
    if(size==1 && key==13) return board_getPower()!=0;
    if(size==1 && key<18) return state.byte[(uint8_t)key];
    return 0;
}
static void set_pullups(uint8_t flags) {
    state.byte[14]=flags;
    /* 关电期间只保存请求，不能重新打开可能反灌的上拉。 */
    if(!board_getPower()) { P0PU=0; P2PU=0; P7PU=0; DisableP6nPullUp(PIN_2); return; }
    P0PU=(flags&1)?0xff:0;
    P2PU=(flags&1)?0xff:0;
    P7PU=(flags&1)?0xff:0;
    if(flags&2) EnableP6nPullUp(PIN_2);
    else DisableP6nPullUp(PIN_2);
}
static void apply_aux_pin(void) {
    if(board_getPower() && ((MODE==1 && (state.byte[17] || state.byte[4]==2)) || (MODE==2 && state.byte[16]))) {
        SetP4nPushPullMode(PIN_5); CART_IRQ=1;
    } else { CART_IRQ=0; SetP4nHighZInputMode(PIN_5); }
}
static bit set_variable(uint8_t size,uint32_t key,uint32_t v) FGX_STACK {
    /* 固件诊断开关，仅供相同固件的只读 A/B 测时；不修改主机设置。 */
    if(size==1 && key==18 && v<=1) { stream_poll_enabled=v!=0; return 1; }
    if(size==1 && key==19 && v<=1) { stream_read_enabled=v!=0; return 1; }
    if(size==1 && key==20 && v<=2) { flashgbx_crc_fast_mode=(uint8_t)v; return 1; }
    if(size==1 && key==21 && v<=1) { flashgbx_program_fast_mode=(uint8_t)v; return 1; }
    if(size==1 && key==22 && v<=1) { flashgbx_rx_burst=v!=0; return 1; }
    if(size==1 && key==24 && v<=1) { dmg_buffer_fast=v!=0; return 1; }
    if(size==1 && key==25 && v<=1) { flashgbx_dmg_pipeline=v!=0; return 1; }
    if(size==1 && key==26 && v<=1) { flashgbx_gbc_batch_fast=v!=0; return 1; }
    if(size==1 && key==27 && v<=1) { flashgbx_gbc_fast_timing=v!=0; return 1; }
    if(size==4 && key<2) {
        if(key==1 && v>32767000UL) return 0;
        state.wide[(uint8_t)key]=v;
        /* 首次 tick 可能紧跟命令完成，额外一周期保证不早于请求的空闲时长。 */
        if(key==1) state.auto_ticks=v?((v+499)/500+1):0;
        return 1;
    }
    if(size==2 && key<7) {
        if(v>65535UL || (key==0 && v>FGX_READ_LIMIT)) return 0;
        state.word[(uint8_t)key]=v;
        if(key==2) state.word[4]=v&255;
        return 1;
    }
    if(size==1 && key<18 && v<=255) {
        if(key==0 && v>2) return 0;
        if(key==0 && v==2 && board_getPower()==2) return 0;
        if(key==0 && v==2) voltage=1;
        if(key==3 && v>12) return 0;
        if(key==13 || ((key==15 || key==16) && v>1)) return 0;
        if(key==14 && v>3) return 0;
        if((key==11 || key==12) && v>2) return 0;
        if(key==4 && v>3) return 0;
        if((key==5 || key==6 || key==10 || key==17) && v>1) return 0;
        if((key==0 || key==2 || key==3 || key==4) && state.byte[(uint8_t)key]!=v) state.configured=0;
        state.byte[(uint8_t)key]=v;
        if(key==14) set_pullups((uint8_t)v);
        if(key==4 || key==16 || key==17) apply_aux_pin();
        return 1;
    }
    return 0;
}
static void read_chunk(uint8_t kind,uint32_t address,uint8_t n) FGX_STACK {
    if(kind==0xb1) {
        if(dmg_buffer_fast && address<0x8000UL && n<=0x8000UL-address)
            cart_gbcRead_block((uint16_t)address,responBuf,n);
        else cart_gbcRead(address,responBuf,n);
    }
    else if(kind==0xc3) cart_ramRead(address,responBuf,n);
    else cart_romRead_data(address,responBuf,((uint16_t)n+1U)>>1);
}
static void read_stream(uint8_t kind) FGX_STACK {
    uint16_t left=TRANSFER;
    uint32_t address=ADDRESS;
    uint8_t n,bus_active=0;
    if(!left || left>FGX_READ_LIMIT || (kind==0xb1?MODE!=1:MODE!=2)) { ack(2); return; }
    /* GBA ROM 高位属于主机逻辑地址；物理窗口仍由卡带的 2/3 寄存器选择。 */
    if(kind==0xc1) address&=0xffffffUL;
    if(address >= (kind==0xc1?0x1000000UL:0x10000UL) ||
       (kind==0xc1?((left+1U)>>1):left) > (kind==0xc1?0x1000000UL:0x10000UL)-address) { ack(2); return; }
    if(kind==0xc1 && left>=64) stream_poll_begin();
    if(kind==0xc1 && stream_read_enabled) {
        while(left>=64) {
            if(flashgbx_cancelled()) goto finished;
            if((uint16_t)ADDRESS>0xffe0U) {
                /* 非对齐请求跨越低 16 位边界时，由原路径重新锁存地址。 */
                if(bus_active) { cart_romRead_stream_end(); bus_active=0; }
                cart_romRead_data(ADDRESS,fgx_read_page,32);
            } else {
                if(!bus_active) { cart_romRead_stream_begin(ADDRESS); bus_active=1; }
                cart_romRead_stream64();
            }
            if(!send_packet(fgx_read_page,64)) goto finished;
            ADDRESS+=32; left-=64;
            if(bus_active && !(uint16_t)ADDRESS) { cart_romRead_stream_end(); bus_active=0; }
        }
        if(bus_active) { cart_romRead_stream_end(); bus_active=0; }
    }
    while(left) {
        if(flashgbx_cancelled()) goto finished;
        n=left>64?64:(uint8_t)left;
        read_chunk(kind,ADDRESS,n);
        if(!send_packet(responBuf,n)) goto finished;
        ADDRESS+=kind==0xc1?(((uint16_t)n+1U)>>1):n;
        left-=n;
    }
    if((TRANSFER&63)==0) send_packet(reply,0);
finished:
    if(bus_active) cart_romRead_stream_end();
    stream_poll_end();
}
static void write_unit(uint32_t address,uint16_t v,bit rom) {
    if(flashgbx_cancelled()) return;
    bus_word[0]=v; bus_word[1]=v>>8;
    if(MODE==1 && rom) {
        if(dmg_buffer_fast && state.byte[4]<=1) cart_gbcWrite_block((uint16_t)address,bus_word,1);
        else fgx_dmg_flash_write(address,v,state.byte[4]);
    }
    else if(MODE==1) cart_gbcWrite(address,bus_word,1);
    else if(rom) cart_romWrite(address,bus_word,1);
    else cart_ramWrite(address,bus_word,1);
}
static uint16_t read_unit(uint32_t address) {
    if(MODE==1) { cart_gbcRead(address,bus_word,1); return bus_word[0]; }
    if(flashgbx_program_fast_mode && state.byte[2]==1 && state.byte[3]==2)
        cart_romRead_data(address,bus_word,1);
    else cart_romRead(address,bus_word,1);
    return ((uint16_t)bus_word[1]<<8)|bus_word[0];
}
static bit wait_program(uint32_t address,uint16_t v) {
    uint16_t attempts=20000,sample;
    while(attempts--) {
        if(flashgbx_cancelled()) return 0;
        sample=read_unit(address);
        if(MODE==1) { sample&=255; v&=255; }
        if(sample==v) return 1;
    }
    state.word[3]=sample;
    return 0;
}
static void flash_command(uint8_t i) {
    if(MODE==1 && dmg_buffer_fast && flashgbx_gbc_batch_fast && state.byte[4]<=1) {
        if(flashgbx_cancelled()) return;
        bus_word[0]=state.commands[i].value;
        cart_gbcWrite_block((uint16_t)state.commands[i].address,bus_word,1);
    } else write_unit(state.commands[i].address,state.commands[i].value,1);
}
static bit program_save(uint8_t type) FGX_STACK;
static bit fixed_method(uint8_t m) { return m==3 || m==4 || m==9 || m==10 || m==11 || m==12; }
static bit program_special(void) FGX_STACK {
    uint8_t method=state.byte[3];
    uint16_t pos,step;
    uint32_t low=ADDRESS&0xffffUL;
    special_failed=0;
    if(!TRANSFER || TRANSFER>FGX_LIMIT) return 0;
    if(method==7) return program_save(1);
    if(method==3 || method==4 || method==9 || method==10 || method==12) {
        if(MODE!=1 || low+TRANSFER>0x10000UL || (method!=9 && TRANSFER%128)) return 0;
    } else if(MODE!=2 || (TRANSFER&1) || ADDRESS>=0x1000000UL || TRANSFER/2>0x1000000UL-ADDRESS) return 0;
    if(method==11 && ADDRESS+TRANSFER/2>0x200000UL) return 0;
    if(method==5 || method==8) {
        if(!BUFFER || BUFFER>FGX_LIMIT || TRANSFER%BUFFER || BUFFER%4) return 0;
    }
    if(method==9) lk_dmg_flash_datel_orbitv2();
    else if(method==6) lk_agb_flash_fujitsu_fastmode();
    else if(method==11) lk_agb_flash_gbamp();
    else {
        step=(method==5 || method==8)?BUFFER:128;
        for(pos=0;pos<TRANSFER && !special_failed && !flashgbx_cancelled();pos+=step) {
            switch(method) {
            case 3: lk_dmg_flash_mmsa(pos); break;
            case 4: lk_dmg_flash_mbc6(pos); break;
            case 5: lk_agb_flash2advance_buffered(step,pos,ADDRESS); break;
            case 8: lk_agb_flash_paged(pos); break;
            case 10: lk_dmg_flash_E201264(pos); break;
            case 12: lk_dmg_flash_bung_16m(pos); break;
            default: return 0;
            }
        }
    }
    if(method==3 || method==4) write_unit((ADDRESS-1)&0xffffUL,0xf0,0);
    if(method==8) write_unit(ADDRESS-1,0xff,1);
    return !special_failed && !flashgbx_cancelled();
}
static bit valid_range(uint32_t length, uint8_t rom) {
    uint32_t limit = MODE==2 && rom ? 0x1000000UL : 0x10000UL;
    uint32_t address=ADDRESS;
    if(MODE!=1 && MODE!=2) return 0;
    /* 只按 ROM 总线的 24 位字地址检查窗口边界，保留 ADDRESS 的逻辑高位和递增语义。 */
    if(MODE==2 && rom) { address&=0xffffffUL; length=(length+1)/2; }
    return address<limit && length<=limit-address;
}
/* 仅转换固件生成的控制值；上传命令与 ROM 字节均保持卡槽位序。
   E8 在 D6/D7 对调后数值不变，使用上位机提供的就绪位识别该映射。 */
static uint16_t intel_wire_value(uint16_t value) {
    if(state.commands[0].value==0xea && ((value^(value>>1))&1)) value^=3;
    if((state.word[5]&0xc0)==0x40 && ((value^(value>>1))&0x40)) value^=0xc0;
    return value;
}
static bit intel_ready(uint32_t address,uint8_t check_errors) {
    uint16_t tries=20000,value,error_mask=state.commands[0].value==0xea?0x39:0x3a;
    if(!state.word[5]) return 0;
    if(state.word[5]&0xff00) error_mask|=0x3a00;
    while(tries--) {
        if(flashgbx_cancelled()) return 0;
        value=read_unit(address); state.word[3]=value;
        /* E8 的 XSR 只使用就绪位；提交后才检查完整错误状态。 */
        if((value&state.word[5])==state.word[6]) return !check_errors || (value&error_mask)==0;
    }
    return 0;
}
static uint16_t page_remaining(uint32_t address,uint16_t buffer,uint8_t unit) FGX_STACK {
    uint16_t mask=buffer-1,offset=(uint16_t)address;
    /* 常见 Flash 页大小为 2 的幂，低 16 位足以计算余数；其它页大小保留原算法。 */
    if(!(buffer&mask)) {
        if(unit==2) offset<<=1;
        return buffer-(offset&mask);
    }
    return buffer-(uint16_t)((address*unit)%buffer);
}
static bit program_intel(void) FGX_STACK {
    uint16_t pos=0,n,j,v;
    uint8_t unit=MODE==2?2:1;
    uint32_t start;
    while(pos<TRANSFER) {
        if(flashgbx_cancelled()) return 0;
        start=ADDRESS;
        if(state.byte[3]==1) {
            if(state.commands[0].value==0x70 || state.commands[0].value==0xb0) {
                write_unit(start,state.commands[0].value,1);
                if(!intel_ready(start,1)) goto failed;
                write_unit(start,state.commands[1].value,1);
            } else write_unit(start,state.commands[0].value,1);
            n=unit;
        } else {
            n=page_remaining(ADDRESS,BUFFER,unit);
            if(n>TRANSFER-pos) n=TRANSFER-pos;
            /* E9/EA 使用至多 512 字缓冲；E8 保留既有上限。 */
            j=MODE==2 && state.commands[0].value!=0xe8?1024:256*unit;
            if(n>j) n=j;
            write_unit(start,intel_wire_value(0x50),1); write_unit(start,0xff,1);
            write_unit(start,state.commands[0].value,1);
            /* G18 E9/EA 在 D0 提交后才切到状态模式，不能拿提交前的阵列值判断就绪。 */
            if(state.commands[0].value==0xe8 && !intel_ready(start,0)) goto failed;
            v=(unit==2?(n>>1):n)-1;
            write_unit(start,intel_wire_value(v),1);
        }
        /* G18 的 WR 不保证递增地址：每个字都重新锁存完整地址。
           与实机 D4 逐字寻址对照一致，不沿用 AMD 卡板的连续写入假设。 */
        for(j=0;j<n;j+=unit) {
            v=cmdBuf[pos+j]; if(unit==2) v|=(uint16_t)cmdBuf[pos+j+1]<<8;
            write_unit(ADDRESS,v,1); ++ADDRESS;
        }
        if(state.byte[3]==2) write_unit(start,0xd0,1);
        if(!intel_ready(start,1)) goto failed;
        write_unit(start,0xff,1);
        if(MODE==2) {
            if(!cart_romVerify_data(start,cmdBuf+pos,n/2)) goto failed;
        } else for(j=0;j<n;j+=unit) {
            if(read_unit(start+j)!=cmdBuf[pos+j]) goto failed;
        }
        pos+=n;
    }
    return 1;
failed:
    write_unit(start,intel_wire_value(0x50),1); write_unit(start,0xff,1); return 0;
}
/* AMD 单字节/字和缓冲编程；擦除指令由主机 D4 发出并轮询。
   未覆盖算法在 A7 返回错误，不能当 SRAM 写后报告编程成功。 */
static bit verify_chunk(uint16_t offset,uint8_t count) {
    uint8_t xdata *expected=cmdBuf+offset;
    uint8_t xdata *actual=responBuf;
    uint8_t value;
    while(count--) {
        value=*actual++;
        if(value!=*expected++) { state.word[3]=value; return 0; }
    }
    return 1;
}
static bit program_flash(void) FGX_STACK {
    uint16_t pos=0,n,unit=MODE==2?2:1,j,v=0;
    uint8_t verify_count;
    uint32_t start,last=ADDRESS;
    if(!state.configured || (state.byte[2]!=1 && state.byte[2]!=2) || (MODE!=1 && MODE!=2)) return 0;
    if(!TRANSFER || TRANSFER>FGX_LIMIT || (MODE==2 && (TRANSFER&1))) return 0;
    if(!valid_range(TRANSFER,1)) return 0;
    if(state.byte[3]==2 && (BUFFER<unit || BUFFER>FGX_LIMIT || BUFFER%unit)) return 0;
    if(state.byte[2]==2) return program_intel();
    /* GBA 不使用 GB 的复位、换银行及片选脉冲标志；跨模式残留不能关闭快路径。
       命令表、ROM 字节与特殊写脚选择仍按原协议处理。 */
    if(MODE==2 && state.byte[3]==1 && flashgbx_program_fast_mode && state.byte[4]<=1)
        return cart_gbaProgram_single();
    /* 只优化原逐字节协议；特殊写脚、银行命令及复位脉冲沿用通用路径。 */
    if(MODE==1 && state.byte[3]==1 && dmg_buffer_fast && flashgbx_gbc_batch_fast &&
       state.byte[4]<=1 && !state.byte[5] && !state.byte[6] && !state.byte[9] &&
       ADDRESS<0x8000UL && TRANSFER<=0x8000UL-ADDRESS) return cart_gbcProgram_single();
    while(pos<TRANSFER) {
        if(flashgbx_cancelled()) return 0;
        start=ADDRESS;
        /* 普通WR的AMD逐字节解锁、编程数据共用一次输出方向设置；
           每个WR/CS脉冲保持原时序，数据发送后释放总线再等待、核对。 */
        cart_gbc_hold_writes=MODE==1 && dmg_buffer_fast && flashgbx_gbc_batch_fast &&
            state.byte[4]<=1 && !state.byte[5] && !state.byte[6] &&
            ADDRESS<0x8000UL;
        if(state.byte[3]==1) {
            if(MODE==1 && state.byte[6]) { special_failed=0; lk_dmg_change_bank(1); }
            flash_command(0); flash_command(1); flash_command(2);
            if(MODE==1 && state.byte[6]) lk_dmg_change_bank(state.word[4]);
            n=unit;
        }
        else {
            n=page_remaining(ADDRESS,BUFFER,unit);
            if(n>TRANSFER-pos) n=TRANSFER-pos;
            if(n>256*unit) n=256*unit;
            flash_command(0); flash_command(1);
            write_unit(start,state.commands[2].value,1);
            v=(unit==2?(n>>1):n)-1;
            if(state.byte[10]) {
                v=amd_wire_byte((uint8_t)v,(uint8_t)state.commands[0].value)|
                    ((uint16_t)amd_wire_byte((uint8_t)v,(uint8_t)(state.commands[0].value>>8))<<8);
            } else v=amd_wire_byte((uint8_t)v,(uint8_t)state.commands[0].value);
            write_unit(start,v,1);
        }
        if(MODE==2 && state.byte[3]==2) {
            /* GBA 缓冲页内地址连续，复用原生 XRAM 写入路径，避免每字重设总线。
               底层每个写脉冲仍检查断线；页边界、确认、轮询及整页核对保持不变。 */
            cart_romWrite_data(ADDRESS,cmdBuf+pos,n/2);
            ADDRESS+=n/2; last=ADDRESS-1;
            v=cmdBuf[pos+n-2]|((uint16_t)cmdBuf[pos+n-1]<<8);
        } else if(MODE==1 && dmg_buffer_fast && state.byte[3]==2 && state.byte[4]<=1 && !state.byte[6] &&
                  ADDRESS<0x8000UL && n<=0x8000UL-ADDRESS) {
            cart_gbcWrite_block((uint16_t)ADDRESS,cmdBuf+pos,n);
            ADDRESS+=n;last=ADDRESS-1;v=cmdBuf[pos+n-1];
        } else for(j=0;j<n;j+=unit) {
            v=cmdBuf[pos+j]; if(unit==2) v|=(uint16_t)cmdBuf[pos+j+1]<<8;
            last=ADDRESS; write_unit(ADDRESS,v,1); ++ADDRESS;
        }
        if(state.byte[3]==2) write_unit(start,state.commands[5].value,1);
        cart_gbcWrite_finish();
        if(MODE==1 && state.byte[5]) {
            lk_delay_us(10); CART_CS2=0; lk_delay_us(3); CART_CS2=1;
            if(state.word[2]) { special_failed=0; lk_dmg_change_bank(state.word[4]); }
        }
        if(!wait_program(last,v)) { write_unit(0,0xf0,1); return 0; }
        /* 末字为 FF 时，无响应或拒绝编程的芯片也可能通过末字轮询。
           按页回读所有数据，确认真实写入后才发送成功 ACK。 */
#if defined(__SDCC) && defined(ROM_WRITE_XDATA_FAST)
        if(MODE==2) {
            if(!cart_romVerify_data(start,cmdBuf+pos,n/2)) {
                state.word[3]=cart_verify_actual; write_unit(0,0xf0,1); return 0;
            }
        } else if(dmg_buffer_fast && start<0x8000UL && n<=0x8000UL-start) {
            if(!cart_gbcVerify_block((uint16_t)start,cmdBuf+pos,n)) {
                state.word[3]=cart_verify_actual;write_unit(0,0xf0,1);return 0;
            }
        } else
#endif
        for(j=0;j<n;j+=verify_count) {
            if(flashgbx_cancelled()) return 0;
            verify_count=n-j>64?64:(uint8_t)(n-j);
            read_chunk(MODE==1?0xb1:0xc1,start+(unit==2?(j>>1):j),verify_count);
            if(!verify_chunk(pos+j,verify_count)) { write_unit(0,0xf0,1); return 0; }
        }
        pos+=n;
    }
    return 1;
}
uint8_t xdata crc_state[4];
#ifdef __SDCC
/* 四个 CRC 字节驻留寄存器，查表时不反复搬运栈变量。
   主循环独占，USB/Timer4 按原有 ISR 约定保存其使用的寄存器。 */
static void crc_block(uint8_t count) __naked {
    (void)count;
    /* SDCC 不推导内嵌汇编的寄存器改写集合，显式保留调用方现场。 */
    __asm
        push acc
        push b
        push dpl
        push dph
        push psw
        push ar0
        push ar1
        push ar2
        push ar3
        push ar4
        push ar5
        push ar6
        push ar7
        mov a,dpl
        jz 00982$
        mov r3,a
        mov dptr,#_crc_state
        movx a,@dptr
        mov r4,a
        inc dptr
        movx a,@dptr
        mov r5,a
        inc dptr
        movx a,@dptr
        mov r6,a
        inc dptr
        movx a,@dptr
        mov r7,a
        mov r0,#<_responBuf
        mov r1,#>_responBuf
    00981$:
        mov dpl,r0
        mov dph,r1
        movx a,@dptr
        inc dptr
        mov r0,dpl
        mov r1,dph
        xrl a,r4
        mov r2,a
        mov dptr,#_crc32_byte0
        movc a,@a+dptr
        xrl a,r5
        mov r4,a
        mov a,r2
        mov dptr,#_crc32_byte1
        movc a,@a+dptr
        xrl a,r6
        mov r5,a
        mov a,r2
        mov dptr,#_crc32_byte2
        movc a,@a+dptr
        xrl a,r7
        mov r6,a
        mov a,r2
        mov dptr,#_crc32_byte3
        movc a,@a+dptr
        mov r7,a
        djnz r3,00981$
        mov dptr,#_crc_state
        mov a,r4
        movx @dptr,a
        inc dptr
        mov a,r5
        movx @dptr,a
        inc dptr
        mov a,r6
        movx @dptr,a
        inc dptr
        mov a,r7
        movx @dptr,a
    00982$:
        pop ar7
        pop ar6
        pop ar5
        pop ar4
        pop ar3
        pop ar2
        pop ar1
        pop ar0
        pop psw
        pop dph
        pop dpl
        pop b
        pop acc
        ret
    __endasm;
}
#else
static void crc_block(uint8_t count) {
    uint8_t i,index;
    for(i=0;i<count;i++) {
        index=crc_state[0]^responBuf[i];
        crc_state[0]=crc_state[1]^crc32_byte0[index];
        crc_state[1]=crc_state[2]^crc32_byte1[index];
        crc_state[2]=crc_state[3]^crc32_byte2[index];
        crc_state[3]=crc32_byte3[index];
    }
}
#endif
static void crc_rom(uint32_t left) FGX_STACK {
    uint32_t address=ADDRESS;
    uint8_t n;
    if(!valid_range(left,1) || left>0x20000UL) { ack(2); return; }
#if defined(__SDCC) && defined(ROM_WRITE_XDATA_FAST)
    if(MODE==2) {
        if(!cart_romCRC32(address,left)) return;
        goto completed;
    }
    if(MODE==1 && dmg_buffer_fast && left && address<0x8000UL && left<=0x8000UL-address) {
        if(!cart_gbcCRC32((uint16_t)address,(uint16_t)left)) return;
        goto completed;
    }
#endif
    crc_state[0]=0xff; crc_state[1]=0xff; crc_state[2]=0xff; crc_state[3]=0xff;
    while(left) {
        if(flashgbx_cancelled()) return;
        n=left>64?64:(uint8_t)left; read_chunk(MODE==1?0xb1:0xc1,address,n);
        crc_block(n);
        address+=MODE==1?n:(((uint16_t)n+1U)>>1); left-=n;
    }
completed:
    reply[0]=~crc_state[3]; reply[1]=~crc_state[2]; reply[2]=~crc_state[1]; reply[3]=~crc_state[0];
    send_packet(reply,4);
}
static bit eeprom_range(uint8_t type) FGX_STACK {
    uint16_t blocks=type==1?64:1024;
    return MODE==2 && board_getPower()==1 && (type==1 || type==2) && TRANSFER && !(TRANSFER&7) &&
           ADDRESS<blocks && TRANSFER/8<=blocks-ADDRESS;
}
static void read_eeprom(uint8_t type) FGX_STACK {
    uint16_t pos;
    uint8_t used=0;
    if(!eeprom_range(type)) { ack(2); return; }
    for(pos=0;pos<TRANSFER;pos+=8) {
        if(flashgbx_cancelled()) return;
        gba_eeprom_read((uint16_t)ADDRESS,responBuf+used,type);
        ++ADDRESS; used+=8;
        if(used==64 || pos+8==TRANSFER) { if(!send_packet(responBuf,used)) return; used=0; }
    }
    if((TRANSFER&63)==0) send_packet(reply,0);
}
static bit write_eeprom(uint8_t type) FGX_STACK {
    uint16_t pos;
    uint8_t i;
    if(!eeprom_range(type)) return 0;
    for(pos=0;pos<TRANSFER;pos+=8) {
        if(flashgbx_cancelled() || !gba_eeprom_write((uint16_t)ADDRESS,cmdBuf+pos,type)) return 0;
        gba_eeprom_read((uint16_t)ADDRESS,responBuf,type);
        for(i=0;i<8;i++) if(responBuf[i]!=cmdBuf[pos+i]) return 0;
        ++ADDRESS;
    }
    return 1;
}
static bit program_save(uint8_t type) FGX_STACK {
    uint16_t pos,n,j,tries;
    uint32_t start;
    if(MODE!=2 || (type!=1 && type!=2) || (type==2 && (TRANSFER%128))) return 0;
    start=type==2?ADDRESS*128:ADDRESS;
    if(start>=65536UL || TRANSFER>65536UL-start) return 0;
    for(pos=0;pos<TRANSFER;pos+=n) {
        if(flashgbx_cancelled()) return 0;
        n=type==1?1:128;
        write_unit(0x5555,0xaa,0); write_unit(0x2aaa,0x55,0); write_unit(0x5555,0xa0,0);
        cart_ramWrite(start+pos,cmdBuf+pos,n);
        tries=20000;
        do {
            if(flashgbx_cancelled()) return 0;
            cart_ramRead(start+pos+n-1,bus_word,1);
            if(bus_word[0]==cmdBuf[pos+n-1]) break;
        } while(--tries);
        if(!tries) { write_unit(0,0xf0,0); return 0; }
        for(j=0;j<n;j++) {
            cart_ramRead(start+pos+j,bus_word,1);
            if(bus_word[0]!=cmdBuf[pos+j]) return 0;
        }
    }
    ADDRESS+=type==2?TRANSFER/128:TRANSFER;
    return 1;
}
static bit dmg_page_write(void) FGX_STACK {
    uint16_t pos,n;
    special_failed=0;
    if(MODE!=1 || !state.configured || !TRANSFER || !BUFFER || BUFFER>256 || !valid_range(TRANSFER,0)) return 0;
    for(pos=0;pos<TRANSFER && !special_failed && !flashgbx_cancelled();pos+=n) {
        n=BUFFER-(uint16_t)(ADDRESS%BUFFER);
        if(n>TRANSFER-pos) n=TRANSFER-pos;
        lk_dmg_eeprom_write_bytes(n,pos,ADDRESS);
        lk_dmg_verify_data(ADDRESS-1,cmdBuf[pos+n-1]);
    }
    return !special_failed && !flashgbx_cancelled();
}
static void read_3d(void) FGX_STACK {
    uint16_t left,n;
    uint32_t bytes_address;
    if(MODE!=2 || !board_getPower() || !TRANSFER || TRANSFER>BUFFER || !BUFFER || BUFFER>4096 || BUFFER%512 || BUFFER%TRANSFER || (TRANSFER&1) || ADDRESS>0x7fffffffUL-BUFFER/2) { ack(2); return; }
    if(memory3d_active && memory3d_offset==BUFFER) {
        CART_CS1=1; ADDRESS+=BUFFER/2; memory3d_active=0;
    }
    if(!memory3d_active) {
        special_failed=0; bytes_address=ADDRESS*2;
        lk_agb_cart_write_short(0x4000c2,(uint16_t)bytes_address);
        lk_agb_cart_write_short(0x4000c3,bytes_address>>16);
        lk_agb_cart_write_short(0x4000c4,0x1000);
        lk_agb_cart_write_short(0x4000c5,0x0800);
        lk_agb_cart_write_short(0x4000c6,BUFFER/512);
        lk_agb_cart_write_short(0x4000c7,0);
        lk_agb_cart_write_short(0x4000c0,0x11);
        lk_agb_cart_write_short(0x4000c1,0);
        lk_delay_us(400);
        cart_gbaRomAddrSetup(0x800); cart_setDirection_ad(0);
        memory3d_offset=0; memory3d_active=1;
    }
    left=TRANSFER;
    while(left && !flashgbx_cancelled()) {
        n=left>64?64:left;
        cart_romRead_continuous(0x800+memory3d_offset/2,responBuf,n/2);
        if(!send_packet(responBuf,n)) return;
        left-=n; memory3d_offset+=n;
    }
    if(!(TRANSFER&63)) send_packet(reply,0);
}
static bit set_pins(uint32_t mask,uint8_t high) FGX_STACK {
    uint8_t index,bits;
    if(mask&0x80000000UL || high>1) return 0;
    if(mask&1) board_setPower(high?voltage:0);
    if(mask&2) P16=high;
    if(mask&4) CART_WR=high;
    if(mask&8) CART_RD=high;
    if(mask&16) CART_CS1=high;
    for(index=0;index<24;index++) if(mask&(1UL<<(index+5))) {
        bits=0x80>>(index&7);
        if(index<16) {
            cart_setDirection_ad(1);
            if(index<8) { if(high) PORT_AD_L|=bits; else PORT_AD_L&=~bits; }
            else { if(high) PORT_AD_H|=bits; else PORT_AD_H&=~bits; }
        } else { cart_setDirection_a(1); if(high) PORT_A|=bits; else PORT_A&=~bits; }
    }
    if(mask&(1UL<<29)) CART_CS2=high;
    if(mask&(1UL<<30)) { SetP4nPushPullMode(PIN_5); CART_IRQ=high; }
    return 1;
}
bit flashgbx_handle(void) FGX_STACK {
    uint16_t len,i,offset;
    uint8_t command,result=1;
    uint8_t saved_ie;
    uint8_t saved_ea;
    uint32_t value;
    /* 原子开始事务，避免 USB 复位与清除总线取消位交错。 */
    saved_ea=EA; EA=0;
    transaction_id=session_id; flashgbx_bus_cancelled=0;
    EA=saved_ea;
    if(!active) return 0;
    if(transport_fault) return 1;
    if(!cmdBuf_i_wr) { idle_poweroff(); return 1; }
    len=expected(); if(cmdBuf_i_wr<len) return 1;
    if(pending) {
        command=pending; pending=0;
        if(command==0xaf) {
            memcpy(&saved_state,cmdBuf,sizeof(state));
            result=1;
            saved_ie=IE2; IE2&=~EUSB;
            if(flashgbx_cancelled()) { IE2=saved_ie; return 1; }
            if(saved_state.word[0]>FGX_READ_LIMIT || saved_state.byte[0]>2 ||
               saved_state.selected_voltage<1 || saved_state.selected_voltage>2 || (saved_state.byte[0]==2 && saved_state.selected_voltage==2) || saved_state.configured>1 ||
               saved_state.byte[14]>3 || saved_state.byte[15]>1 || saved_state.byte[16]>1 ||
               saved_state.wide[1]>32767000UL ||
               saved_state.bank_count>3 ||
               (saved_state.configured && (saved_state.byte[2]>2 || saved_state.byte[3]<1 || saved_state.byte[3]>12)) || saved_state.byte[4]>3) {
                memset(&saved_state,0,sizeof(saved_state)); saved_state.selected_voltage=1;
                result=2;
            }
            memcpy(&state,&saved_state,sizeof(state));
            state.auto_ticks=state.wide[1]?((state.wide[1]+499)/500+1):0;
            set_pullups(state.byte[14]);
            apply_aux_pin();
            IE2=saved_ie;
            ack(result);
        } else {
            if(!board_getPower()) result=2;
            else if(command==0xd3) {
                if(state.byte[3]>=3 && (state.configured || fixed_method(state.byte[3]) || state.byte[3]==7)) result=program_special()?1:2;
                else result=program_flash()?1:2;
            }
            else if(command==0xc6) result=write_eeprom(pending_type)?1:2;
            else if(command==0xc7) result=program_save(pending_type)?1:2;
            else if(command==0xb6 && MODE==1 && TRANSFER<=32 && !(TRANSFER&1) && ADDRESS+TRANSFER/2<=256) {
                special_failed=0; lk_dmg_mbc7_write_eeprom(); result=special_failed?2:1;
            }
            else if(command==0xb7 && MODE==1 && valid_range(TRANSFER,0)) {
                special_failed=0; lk_dmg_mbc6_mmsa_write_flash_bytes(); result=special_failed?2:1;
            }
            else if(command==0xb9) result=dmg_page_write()?1:2;
            else if(command==0xb3 && MODE==1 && valid_range(TRANSFER,0)) { cart_gbcWrite(ADDRESS,cmdBuf,TRANSFER); ADDRESS+=TRANSFER; }
            else if(command==0xc4 && MODE==2 && valid_range(TRANSFER,0)) { cart_ramWrite(ADDRESS,cmdBuf,TRANSFER); ADDRESS+=TRANSFER; }
            else result=2;
            ack(result);
        }
        consume(len); return 1;
    }
    command=cmdBuf[0];
    if(memory3d_active && command!=0xc8) {
        CART_CS1=1; cart_setDirection_a(0); cart_setDirection_ad(0);
        ADDRESS+=BUFFER/2; memory3d_active=0;
        if(command==0) { consume(len); return 1; }
    }
    /* 流式读取允许 8 KiB；所有跟随原始输入载荷的写命令仍限制为接收缓冲可容纳的 4 KiB。 */
    if(TRANSFER>FGX_LIMIT && (command==0xb3 || command==0xc4 || command==0xd3 || command==0xc6 ||
       command==0xc7 || command==0xb6 || command==0xb7 || command==0xb9)) {
        ack(2);transport_fault=1;consume(len);return 1;
    }
    if(!board_getPower() && (command==0xb1 || command==0xb2 || command==0xc1 || command==0xc2 ||
       command==0xc3 || command==0xca || command==0xd1 || command==0xd2 || command==0xd4 || command==0xd5)) {
        ack(2); consume(len); return 1;
    }
    switch(command) {
    case 0xa0:
        for(i=0;i<20;i++) { P16=1; P16=0; P16=1; P16=0; } ack(1); break;
    case 0x55:
        if(cmdBuf[1]==0xaa) {
            for(i=0;i<sizeof(probe_reply)-1;i++) reply[i]=probe_reply[i];
            send_packet(reply,sizeof(probe_reply)-1);
        } else ack(2);
        break;
    case 0x4c: ack(cmdBuf[1]==0x4b?0xff:2); break;
    case 0xa1: query_info(); break;
    case 0xa2:
        value=(MODE!=2 || !board_getPower());
        if(MODE!=2) state.configured=0;
        MODE=2; voltage=1; select_power(voltage); apply_aux_pin(); special_failed=0;
        if(!cart_reverse_table_prepare()) { ack(2); break; }
        if(value) lk_agb_bootup_sequence();
        ack(special_failed?2:1); break;
    case 0xa3:
        /* 烧录流程先选电压再重复选择 GB 模式；同模式命令必须保留已选电压。 */
        if(MODE!=1) { state.configured=0; voltage=2; }
        MODE=1; select_power(voltage); apply_aux_pin(); ack(1); break;
    case 0xa4: voltage=1; select_power(voltage); ack(1); break;
    case 0xa5: if(MODE==2) { ack(2); break; } voltage=2; select_power(voltage); ack(1); break;
    case 0xa6: ack(set_variable(cmdBuf[1],be32(2),be32(6))?1:2); break;
    case 0xad: put32(get_variable(cmdBuf[1],be32(2))); send_packet(reply,4); break;
    case 0xa8:
        CART_RD=1; CART_WR=1; CART_CS1=1; CART_CS2=1;
        cart_setDirection_ad(0); cart_setDirection_a(0); ack(1); break;
    case 0xa9:
        value=be32(1);
        while(value-- && !flashgbx_cancelled()) { P16=1; _nop_(); P16=0; _nop_(); } ack(1); break;
    case 0xab: set_pullups(1); ack(1); break;
    case 0xac: set_pullups(0); ack(1); break;
    case 0xb1: case 0xc1: case 0xc3: read_stream(command); break;
    case 0xb5:
        if(MODE!=1 || !board_getPower() || !TRANSFER || TRANSFER>32 || (TRANSFER&1) || ADDRESS+TRANSFER/2>256) { ack(2); break; }
        special_failed=0; lk_dmg_mbc7_read_eeprom(); break;
    case 0xc8: read_3d(); break;
    case 0xc5: read_eeprom(cmdBuf[1]); break;
    case 0xc6: case 0xc7:
        if(!TRANSFER || TRANSFER>FGX_LIMIT) { ack(2); transport_fault=1; }
        else { pending=command; pending_type=cmdBuf[1]; }
        break;
    case 0xca:
        if(MODE!=2) { ack(2); break; }
        gba_rtc_read(reply); send_packet(reply,8); break;
    case 0xb2: case 0xd1:
        if(MODE!=1) ack(2); else { write_unit(be32(1),cmdBuf[5],command==0xd1); ack(1); } break;
    case 0xc2: case 0xd2:
        if(MODE!=2) ack(2); else { write_unit(be32(1),be16(5),1); ack(1); } break;
    case 0xb3: case 0xc4: case 0xd3: case 0xb6: case 0xb7: case 0xb9:
        /* 长度无效时无法定位后续数据边界，必须隔离到重连，避免把数据当写命令。 */
        if(!TRANSFER || TRANSFER>FGX_LIMIT) { ack(2); transport_fault=1; }
        else pending=command;
        break;
    case 0xd4:
        if(MODE!=1 && MODE!=2) { ack(2); break; }
        for(i=0;i<cmdBuf[2];i++) { offset=3+i*6; write_unit(be32(offset),be16(offset+4),cmdBuf[1]!=0); }
        ack(1); break;
    case 0xa7:
        state.configured=0;
        if(cmdBuf[1]>2 || cmdBuf[2]<1 || cmdBuf[2]>12 || cmdBuf[3]>3) { ack(2); break; }
        if(cmdBuf[1]==1 && cmdBuf[2]<=2 && !amd_commands_valid()) { ack(2); break; }
        if(cmdBuf[1]==2 && cmdBuf[2]<=2 && ((cmdBuf[2]==2 && ((be16(8)!=0xe8 && be16(8)!=0xe9 && be16(8)!=0xea) || be16(26)!=0xd0)) ||
           (cmdBuf[2]==1 && be16(8)!=0x40 && be16(8)!=0x10 && !((be16(8)==0x70 && (be16(14)==0x10 || be16(14)==0x40)) || (be16(8)==0xb0 && be16(14)==0x10))))) { ack(2); break; }
        state.byte[2]=cmdBuf[1]; state.byte[3]=cmdBuf[2]; state.byte[4]=cmdBuf[3];
        for(i=0;i<6;i++) { offset=4+i*6; state.commands[i].address=be32(offset); state.commands[i].value=be16(offset+4); }
        apply_aux_pin();
        state.configured=1; ack(1); break;
    case 0xb8:
        if(cmdBuf[1]>3) { ack(2); break; }
        for(i=0;i<cmdBuf[1];i++) if(cmdBuf[6+i*5]>1 ||
           be32(2+i*5)>(cmdBuf[6+i*5]?255UL:65535UL)) result=2;
        if(result==2) { ack(2); break; }
        state.bank_count=cmdBuf[1];
        for(i=0;i<state.bank_count;i++) { state.bank_address[i]=be32(2+i*5); state.bank_mode[i]=cmdBuf[6+i*5]; }
        ack(1); break;
    case 0xb4:
        /* FlashGBX 的卡带分析在 AGB 模式也会发送 B4 清理。
           /RD、/WR 保持高电平，GBA 的 /CS2 脉冲不会形成存档写周期。 */
        CART_RD=1; CART_WR=1;
        CART_CS2=0;
        lk_delay_us(5000);
        CART_CS2=1; ack(1); break;
    case 0xc9:
        if(MODE!=2) { ack(2); break; }
        special_failed=0; lk_agb_bootup_sequence(); ack(special_failed?2:1); break;
    case 0xd5: crc_rom(be32(1)); break;
    case 0xae:
        for(i=0;i<sizeof(state);i+=64) { offset=sizeof(state)-i; if(!send_packet((uint8_t xdata *)&state+i,offset>64?64:offset)) break; }
        break;
    case 0xaf: pending=command; break;
    case 0xf2:
        value=!board_getPower(); select_power(voltage); apply_aux_pin();
        if(value && MODE==2) { special_failed=0; lk_agb_bootup_sequence(); }
        ack(1); break;
    case 0xf3:
        CART_CS1=1; CART_CS2=1; CART_RD=1; CART_WR=1;
        cart_setDirection_a(0); cart_setDirection_ad(0); board_setPower(0); apply_aux_pin(); ack(1); break;
    case 0xf4: ack(board_getPower()!=0); break;
    case 0xfe: ack((uint8_t)~cmdBuf[1]); break;
    case 0xf5: ack(set_pins(be32(1),cmdBuf[5])?1:2); break;
    case 0xf6: ack(0); break; /* 没有安装实体模式/插卡检测开关，握手也不声明这些能力。 */
    case 0x30: ack(1); break;
    case 0xf1:
        /* 未支持的流式命令可能紧跟原始数据，隔离到重连，避免当作新命令执行。 */
        ack(2); transport_fault=1; break;
    /* 特殊卡带、EEPROM、RTC、bootloader 需独立实现，禁止伪造成功。 */
    default: ack(2); break;
    }
    consume(len); return 1;
}
#endif
