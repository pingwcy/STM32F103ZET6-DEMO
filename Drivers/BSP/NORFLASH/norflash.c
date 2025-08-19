#include <string.h>
#include "../../BSP/SPI/spi.h"
extern void delay_us(uint32_t nus);
extern void delay_ms(uint16_t nms);
#include "../../BSP/NORFLASH/norflash.h"

/* ========= 芯片类型 ========= */
uint16_t g_norflash_type = NM25Q128;     /* 默认 NM25Q128 */

/* ========= 4KB 扇区缓存 ========= */
static uint8_t  g_cache_buf[4096];             /* 4KB 缓存 */
static uint32_t g_cache_sector4k = 0xFFFFFFFF;  /* 当前缓存的4KB扇区号（逻辑序号） */
static uint8_t  g_cache_valid    = 0;          /* 缓存是否已加载 */
static uint8_t  g_cache_dirty    = 0;          /* 缓存是否被修改 */

/* ========= 低层原始读（直通硬件，仅供缓存装载等内部使用，避免递归） ========= */
static void norflash_read_raw(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t i;
    NORFLASH_CS(0);
    spi2_read_write_byte(FLASH_ReadData);
    /* 发送32/24位地址 */
    if (g_norflash_type == W25Q256)
        spi2_read_write_byte((uint8_t)((addr)>>24));
    spi2_read_write_byte((uint8_t)((addr)>>16));
    spi2_read_write_byte((uint8_t)((addr)>>8));
    spi2_read_write_byte((uint8_t)addr);
    /* 读取 */
    for (i = 0; i < datalen; i++)
        pbuf[i] = spi2_read_write_byte(0xFF);
    NORFLASH_CS(1);
}

/* ========= 常规底层工具 ========= */
static void norflash_wait_busy(void)
{
    while ((norflash_read_sr(1) & 0x01) == 0x01);   /* 等待BUSY清零 */
}

void norflash_write_enable(void)
{
    NORFLASH_CS(0);
    spi2_read_write_byte(FLASH_WriteEnable);
    NORFLASH_CS(1);
}

static void norflash_send_address(uint32_t address)
{
    if (g_norflash_type == W25Q256)
        spi2_read_write_byte((uint8_t)((address)>>24));
    spi2_read_write_byte((uint8_t)((address)>>16));
    spi2_read_write_byte((uint8_t)((address)>>8));
    spi2_read_write_byte((uint8_t)address);
}

uint8_t norflash_read_sr(uint8_t regno)
{
    uint8_t byte = 0, command = 0;

    switch (regno)
    {
        case 1: command = FLASH_ReadStatusReg1; break;
        case 2: command = FLASH_ReadStatusReg2; break;
        case 3: command = FLASH_ReadStatusReg3; break;
        default: command = FLASH_ReadStatusReg1; break;
    }

    NORFLASH_CS(0);
    spi2_read_write_byte(command);
    byte = spi2_read_write_byte(0Xff);
    NORFLASH_CS(1);
    return byte;
}

void norflash_write_sr(uint8_t regno, uint8_t sr)
{
    uint8_t command = 0;

    switch (regno)
    {
        case 1: command = FLASH_WriteStatusReg1; break;
        case 2: command = FLASH_WriteStatusReg2; break;
        case 3: command = FLASH_WriteStatusReg3; break;
        default: command = FLASH_WriteStatusReg1; break;
    }

    NORFLASH_CS(0);
    spi2_read_write_byte(command);
    spi2_read_write_byte(sr);
    NORFLASH_CS(1);
}

uint16_t norflash_read_id(void)
{
    uint16_t deviceid;
    NORFLASH_CS(0);
    spi2_read_write_byte(FLASH_ManufactDeviceID);
    spi2_read_write_byte(0);
    spi2_read_write_byte(0);
    spi2_read_write_byte(0);
    deviceid = spi2_read_write_byte(0xFF) << 8;
    deviceid |= spi2_read_write_byte(0xFF);
    NORFLASH_CS(1);
    return deviceid;
}

/* ========= 初始化 ========= */
void norflash_init(void)
{
    uint8_t temp;

    NORFLASH_CS_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct;
    gpio_init_struct.Pin = NORFLASH_CS_GPIO_PIN;
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init_struct.Pull = GPIO_PULLUP;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(NORFLASH_CS_GPIO_PORT, &gpio_init_struct);

    NORFLASH_CS(1);

    spi2_init();
    spi2_set_speed(SPI_SPEED_2);        /* 18MHz */

    g_norflash_type = norflash_read_id();

    /* W25Q256 使能4字节地址 */
    if (g_norflash_type == W25Q256)
    {
        temp = norflash_read_sr(3);
        if ((temp & 0x01) == 0)         /* 未开启4字节地址模式则开启 */
        {
            norflash_write_enable();
            temp |= 1 << 1;             /* ADP=1 */
            norflash_write_sr(3, temp);

            NORFLASH_CS(0);
            spi2_read_write_byte(FLASH_Enable4ByteAddr);
            NORFLASH_CS(1);
        }
    }

    /* 缓存失效 */
    g_cache_sector4k = 0xFFFFFFFF;
    g_cache_valid = 0;
    g_cache_dirty = 0;
}

/* ========= 页写 / 无校验写（与原版本一致） ========= */
static void norflash_write_page(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t i;

    norflash_write_enable();

    NORFLASH_CS(0);
    spi2_read_write_byte(FLASH_PageProgram);
    norflash_send_address(addr);
    for (i = 0; i < datalen; i++)
        spi2_read_write_byte(pbuf[i]);
    NORFLASH_CS(1);
    norflash_wait_busy();
}

void norflash_write_nocheck(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    uint16_t pageremain = 256 - (addr % 256);
    if (datalen <= pageremain) pageremain = datalen;

    while (1)
    {
        norflash_write_page(pbuf, addr, pageremain);
        if (datalen == pageremain) break;

        pbuf    += pageremain;
        addr    += pageremain;
        datalen -= pageremain;
        pageremain = (datalen > 256) ? 256 : datalen;
    }
}

/* ========= 缓存管理（4KB一致性） ========= */

/* 将指定4KB扇区加载到缓存（必要时先刷新旧缓存） */
static void norflash_cache_load(uint32_t sector4k_index)
{
    if (g_cache_valid && g_cache_sector4k == sector4k_index)
        return;

    /* 切换扇区前先落盘旧缓存 */
    if (g_cache_valid && g_cache_dirty)
    {
        /* 刷旧缓存 */
        uint32_t base = g_cache_sector4k * 4096U;
        norflash_erase_sector(g_cache_sector4k);                 /* 擦4KB（接口按扇区号） */
        norflash_write_nocheck(g_cache_buf, base, 4096U);        /* 整扇区写回 */
        g_cache_dirty = 0;
    }

    /* 载入新扇区 */
    norflash_read_raw(g_cache_buf, sector4k_index * 4096U, 4096U);
    g_cache_sector4k = sector4k_index;
    g_cache_valid = 1;
    g_cache_dirty = 0;
}

/* 将当前缓存（若脏）写回 */
static void norflash_cache_flush(void)
{
    if (!g_cache_valid || !g_cache_dirty) return;

    uint32_t base = g_cache_sector4k * 4096U;
    norflash_erase_sector(g_cache_sector4k);
    norflash_write_nocheck(g_cache_buf, base, 4096U);
    g_cache_dirty = 0;
}

/* ========= 上层透明读写 ========= */
void norflash_read(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    while (datalen)
    {
        uint32_t sector4k_index = addr / 4096U;
        uint16_t offset_in_4k   = addr % 4096U;
        uint16_t chunk          = 4096U - offset_in_4k;
        if (chunk > datalen) chunk = datalen;

        norflash_cache_load(sector4k_index);
        memcpy(pbuf, g_cache_buf + offset_in_4k, chunk);

        pbuf    += chunk;
        addr    += chunk;
        datalen -= chunk;
    }
}

void norflash_write(uint8_t *pbuf, uint32_t addr, uint16_t datalen)
{
    while (datalen)
    {
        uint32_t sector4k_index = addr / 4096U;
        uint16_t offset_in_4k   = addr % 4096U;
        uint16_t chunk          = 4096U - offset_in_4k;
        if (chunk > datalen) chunk = datalen;

        norflash_cache_load(sector4k_index);
        memcpy(g_cache_buf + offset_in_4k, pbuf, chunk);
        g_cache_dirty = 1;

        pbuf    += chunk;
        addr    += chunk;
        datalen -= chunk;
    }
}

/* 手动同步：在掉电前/关键时刻调用 */
void norflash_sync(void)
{
    norflash_cache_flush();
}

/* ========= 擦除（保持原有“参数是4KB扇区号”的语义），并处理缓存一致性 ========= */
void norflash_erase_chip(void)
{
    /* 先落盘并失效缓存 */
    norflash_sync();
    g_cache_valid = 0;
    g_cache_dirty = 0;
    g_cache_sector4k = 0xFFFFFFFF;

    norflash_write_enable();
    norflash_wait_busy();
    NORFLASH_CS(0);
    spi2_read_write_byte(FLASH_ChipErase);
    NORFLASH_CS(1);
    norflash_wait_busy();
}

/* saddr：4KB 扇区号（不是字节地址！）*/
void norflash_erase_sector(uint32_t saddr)
{
    /* 若当前缓存正好是这个扇区，先落盘/失效（此处直接失效，因为调用者显式要擦除） */
    if (g_cache_valid && g_cache_sector4k == saddr)
    {
        g_cache_valid = 0;
        g_cache_dirty = 0;
        g_cache_sector4k = 0xFFFFFFFF;
    }

    uint32_t addr = saddr * 4096U; /* 转字节地址 */
    norflash_write_enable();
    norflash_wait_busy();

    NORFLASH_CS(0);
    spi2_read_write_byte(FLASH_SectorErase);
    norflash_send_address(addr);
    NORFLASH_CS(1);
    norflash_wait_busy();
}
