// SPDX-License-Identifier: GPL-2.0-only
/*
 * inf1000.c  --  Densitron Audio ALSA SoC driver with FPGA integration
 *
 * Copyright 2024 Densitron/Nexteq
 *
 * This driver integrates:
 * - INF1000 codec for audio capture
 * - TAS5733L power amplifier (I2C)
 * - PCM1748 headphone DAC (SPI)
 * - FPGA control (SPI) for audio routing, rotary encoders, RGB LEDs
 * - Interrupt-driven headphone detection
 *
 * FPGA REQUIREMENTS:
 * - Master clock: 46.08 MHz (fixed)
 * - Format: S24_LE Left Justified ONLY
 * - Channels: Stereo (2 channels)
 * - Sample rates: 8kHz - 48kHz
 *
 * Author: Gian Luca Bernocchi <gianluca.bernocchi@quixant.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/leds.h>
#include <linux/workqueue.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/jack.h>


/* Hardware Configuration Constants */
#define DENSITRON_MCLK_FREQ          46080000  /* Fixed FPGA MCLK */
#define DENSITRON_MIN_RATE           8000
#define DENSITRON_MAX_RATE           48000
#define DENSITRON_CHANNELS           2         /* Stereo only */
#define DENSITRON_FORMAT             SNDRV_PCM_FORMAT_S24_LE
#define DENSITRON_PHYSICAL_WIDTH     32

/* Supported sample rates (must be exact for FPGA) */
static const unsigned int densitron_sample_rates[] = {
    8000, 16000, 32000, 48000
};

static const struct snd_pcm_hw_constraint_list densitron_rate_constraints = {
    .count = ARRAY_SIZE(densitron_sample_rates),
    .list = densitron_sample_rates,
    .mask = 0,
};

/* FPGA Register Definitions */
#define FPGA_REG_LED1_R              0x00
#define FPGA_REG_LED1_G              0x01
#define FPGA_REG_LED1_B              0x02
#define FPGA_REG_LED2_R              0x03
#define FPGA_REG_LED2_G              0x04
#define FPGA_REG_LED2_B              0x05
#define FPGA_REG_ENCODER0            0x06  /* Right rotary - delta value */
#define FPGA_REG_ENCODER1            0x07  /* Left rotary - delta value */
#define FPGA_REG_STATUS              0x08  /* Signal status (read-only) */
#define FPGA_REG_INT_STATUS          0x09  /* Interrupt status (R/W - write 1 to clear then 0 for reset) */
#define FPGA_REG_SYSCTRL             0x0A  /* System control - audio routing */
#define FPGA_REG_HEADSET_MIC_GAIN    0x0B
#define FPGA_REG_FRONT_MIC_GAIN      0x0C
#define FPGA_REG_HP_THRESHOLD        0x0D
#define FPGA_REG_INT_MASK            0x0E  /* Interrupt masking */
#define FPGA_REG_FW_VERSION          0x0F  /* FPGA firmware version (read-only) */
#define FPGA_REG_PCB_VERSION         0x10  /* PCB version (read-only) */
#define FPGA_REG_AUDIO_CTRL0         0x11  /* Audio control */
#define FPGA_REG_AUDIO_STATUS0       0x12  /* Audio status (read-only) */

/* Legacy aliases for compatibility */
#define FPGA_REG_LEFT_ROTARY         FPGA_REG_ENCODER1
#define FPGA_REG_RIGHT_ROTARY        FPGA_REG_ENCODER0

/* FPGA Status Register Bits (0x08) - Read-only */
#define FPGA_STATUS_BUTTON0          BIT(0)  /* btn0 signal */
#define FPGA_STATUS_BUTTON1          BIT(2)  /* btn1 signal */
#define FPGA_STATUS_HP_DETECT        BIT(4)  /* hedest - 1 = jack inserted */
#define FPGA_STATUS_POE_PWR          BIT(5)  /* poe_pwr */
#define FPGA_STATUS_AUX_PWR          BIT(6)  /* aux_pwr */

/* FPGA Interrupt Status Register Bits (0x09) - Read to check, Write 0 to clear */
#define FPGA_INT_BUTTON0             BIT(0)  /* btn0 activity */
#define FPGA_INT_ENCODER0            BIT(1)  /* slide0 (encoder 0 changed) */
#define FPGA_INT_BUTTON1             BIT(2)  /* btn1 activity */
#define FPGA_INT_ENCODER1            BIT(3)  /* slide1 (encoder 1 changed) */
#define FPGA_INT_HP_DETECT           BIT(4)  /* headset detection changed */

/* FPGA Interrupt Mask Register Bits (0x0E) - Write 1 to mask (disable) interrupt */
#define FPGA_INT_MSK_BUTTON0         BIT(0)  /* btn0 */
#define FPGA_INT_MSK_ENCODER0        BIT(1)  /* slide0 */
#define FPGA_INT_MSK_BUTTON1         BIT(2)  /* btn1 */
#define FPGA_INT_MSK_ENCODER1        BIT(3)  /* slide1 */
#define FPGA_INT_MSK_HP_DETECT       BIT(4)  /* hedest */

/* FPGA System Control Register Bits (0x0A) */
#define FPGA_SYSCTRL_SYS_RST         BIT(0)  /* System soft reset */
#define FPGA_SYSCTRL_ENC0_RST        BIT(1)  /* Encoder 0 reg reset */
#define FPGA_SYSCTRL_ENC1_RST        BIT(2)  /* Encoder 1 reg reset */
#define FPGA_SYSCTRL_SAI_LOOP        BIT(3)  /* SAI RX/TX internal loop (debug only) */
#define FPGA_SYSCTRL_SPEAKER_EN      BIT(4)  /* Enable speaker I2S channel */
#define FPGA_SYSCTRL_HEADSET_EN      BIT(5)  /* Enable headset I2S channel */
#define FPGA_SYSCTRL_FRONT_MIC_EN    BIT(6)  /* Enable default microphone I2S channel */
#define FPGA_SYSCTRL_HEADSET_MIC_EN  BIT(7)  /* Enable headset microphone I2S channel */

/* AUDIO_CTRL0 Register Bits (0x11) */
#define FPGA_AUDIO_12V_PD            BIT(7)  /* 12V power down (active low) */
#define FPGA_AUDIO_SPKR_FMT          BIT(2)  /* PCM1808 FMT */
#define FPGA_AUDIO_SPKR_MD1          BIT(1)  /* PCM1808 MD1 */
#define FPGA_AUDIO_SPKR_MD0          BIT(0)  /* PCM1808 MD0 */

/* AUDIO_STATUS0 Register Bits (0x12) */
#define FPGA_AUDIO_DAC_ZEROR         BIT(1)  /* PCM1748 zero detect right */
#define FPGA_AUDIO_DAC_ZEROL         BIT(0)  /* PCM1748 zero detect left */


/* FPGA Commands */
#define FPGA_CMD_WRITE               0xFF
#define FPGA_CMD_READ                0xFE

/* TAS5733L Power Amplifier Registers */
#define TAS5733_CLOCK_CTRL           0x00
#define TAS5733_DEVICE_ID            0x01
#define TAS5733_ERROR_STATUS         0x02
#define TAS5733_SYS_CTRL1            0x03
#define TAS5733_AUDIO_FORMAT         0x04
#define TAS5733_SYS_CTRL2            0x05
#define TAS5733_SOFT_MUTE            0x06
#define TAS5733_MASTER_VOL           0x07
#define TAS5733_LEFT_VOL             0x08
#define TAS5733_RIGHT_VOL            0x09
#define TAS5733_TRIM_REG             0x1B
#define TAS5733_MOD_SCHEME           0x20
#define TAS5733_ICD1                 0x11
#define TAS5733_ICD2                 0x12
#define TAS5733_ICD3                 0x13
#define TAS5733_ICD4                 0x14

/* PCM1748 Headphone DAC Registers */
#define PCM1748_LEFT_VOLUME          0x10  /* Reg 16 - Attenuation: 0.5*(Val-255) dB */
#define PCM1748_RIGHT_VOLUME         0x11  /* Reg 17 - Attenuation: 0.5*(Val-255) dB */
#define PCM1748_SOFT_MUTE            0x12  /* Reg 18 - bit0: L mute, bit1: R mute */
#define PCM1748_DAC_CTRL             0x13  /* Reg 19 - DAC control and de-emphasis */
#define PCM1748_FORMAT_CTRL          0x14  /* Reg 20 - Audio format selection */

/* Microphone source selection */
enum mic_source {
    MIC_SOURCE_NONE = 0,
    MIC_SOURCE_FRONT = 1,
    MIC_SOURCE_HEADSET = 2,
    MIC_SOURCE_BOTH = 3,  
};


/* Driver Private Data Structure */
struct densitron_audio_priv {
    /* Core components */
    struct snd_soc_component *component;
    struct i2c_client *tas5733_client;
    struct spi_device *fpga_spi;
    struct spi_device *pcm1748_spi;
    
    /* GPIO and IRQ */
    struct gpio_desc *reset_gpio;
    struct gpio_desc *fpga_irq_gpio;
    int fpga_irq;
    
    /* Input devices for rotary encoders */
    struct input_dev *left_rotary;
    struct input_dev *right_rotary;
    
    /* LED devices */
    struct led_classdev led1_r, led1_g, led1_b;
    struct led_classdev led2_r, led2_g, led2_b;
    
    /* Audio routing state */
    unsigned int mclk;
    unsigned int sample_rate;
    bool headphones_connected;
    bool audio_path_local;      /* true=local speaker, false=headphones */
    enum mic_source mic_source; /* Current microphone source */
    bool auto_mic_switch;       /* Auto-switch mic on headset insertion */
    bool hardware_initialized;
    
    /* Work queue for interrupt handling */
    struct work_struct irq_work;
    struct mutex hw_lock;  /* Protect hardware access */
    
    /* Last rotary encoder values */
    int left_rotary_last;
    int right_rotary_last;
    
    /* Current volume levels (cached) */
    unsigned int speaker_left_vol;
    unsigned int speaker_right_vol;
    unsigned int hp_left_vol;
    unsigned int hp_right_vol;
    unsigned int headset_mic_gain;
    unsigned int front_mic_gain;
    
    /* PCM1748 register cache (write-only chip) */
    unsigned int pcm1748_mute;
    unsigned int pcm1748_dac_ctrl;
    unsigned int pcm1748_format;
    
    /* Jack detection */
    struct snd_soc_jack headphone_jack;
    
    /* Debug counters */
    atomic_t irq_count;
    atomic_t format_rejects;
    
    /* Rotary encoder accumulators */
    int left_rotary_position;   /* Accumulated position for encoder1 */
    int right_rotary_position;  /* Accumulated position for encoder0 */
};

/**
 * Microphone Gain Specifications:
 * - Hardware register: 0x00 = Maximum gain, 0xFF = Minimum gain (inverted)
 * - Gain range: +20dB to -107dB (127dB total range)
 * - Resolution: 0.5dB per step (255 steps)
 * 
 * Formula:
 *   Gain(dB) = 20 - (register_value / 2)
 *   Register = (20 - Gain(dB)) * 2
 */

#define MIC_GAIN_MAX_DB        20      /* Maximum gain: +20dB (register 0x00) */
#define MIC_GAIN_MIN_DB        -107    /* Minimum gain: -107.5dB (register 0xFF) */
#define MIC_GAIN_STEP_DIVISOR  2       /* 0.5dB per step */
#define MIC_GAIN_DEFAULT_DB    0       /* Default: 0dB (register 0x28 = 40) */

/**
 * mic_gain_db_to_hw - Convert dB gain to hardware register value
 * @gain_db: Gain in dB (MIC_GAIN_MIN_DB to MIC_GAIN_MAX_DB)
 * 
 * Returns: Hardware register value (0x00 to 0xFF)
 * 
 * Hardware: 0x00 = +20dB (max), 0xFF = -107.5dB (min)
 * Each step is 0.5dB
 */
static u8 mic_gain_db_to_hw(int gain_db)
{
    int reg_value;
    
    /* Clamp to valid range */
    if (gain_db > MIC_GAIN_MAX_DB)
        gain_db = MIC_GAIN_MAX_DB;
    if (gain_db < MIC_GAIN_MIN_DB)
        gain_db = MIC_GAIN_MIN_DB;
    
    /* Convert: register = (20 - gain_db) * 2 
     * CRITICAL: Parentheses matter! */
    reg_value = (MIC_GAIN_MAX_DB - gain_db) * MIC_GAIN_STEP_DIVISOR;
    
    /* Clamp to 0-255 range */
    if (reg_value < 0)
        reg_value = 0;
    if (reg_value > 255)
        reg_value = 255;
    
    return (u8)reg_value;
}

/**
 * mic_gain_hw_to_db - Convert hardware register value to dB gain
 * @hw_value: Hardware register value (0x00 to 0xFF)
 * 
 * Returns: Gain in dB
 * 
 * Hardware: 0x00 = +20dB (max), 0xFF = -107.5dB (min)
 */
static int mic_gain_hw_to_db(u8 hw_value)
{
    
    /* Convert: gain_db = 20 - (register * 0.5) */
    return MIC_GAIN_MAX_DB - (int)(hw_value / MIC_GAIN_STEP_DIVISOR);
}



/* ============================================================================
 * Volume Mapping and Conversion Functions
 * ============================================================================ */

/* TAS5733L Volume Mapping Table
 * Based on measured output power vs register values
 * Register values are INVERTED (higher = quieter)
 */
struct volume_map_point {
    unsigned int percent;      /* 0-100 */
    unsigned int power_mw;     /* Power in mW */
    unsigned int reg_value;    /* TAS5733L register value */
};

static const struct volume_map_point tas5733_volume_map[] = {
    { 100, 5120, 139 },   /* 5.12W at 100% */
    { 80,  4096, 165 },   /* 4.096W at 80% */
    { 60,  3072, 191 },   /* 3.072W at 60% */
    { 40,  2048, 218 },   /* 2.048W at 40% */
    { 20,  1024, 244 },   /* 1.024W at 20% */
    { 10,   512, 270 },   /* 0.512W at 10% */
    { 5,    256, 297 },   /* 0.256W at 5% */
    { 1,      0, 350 },   /* 0W at 1% (near mute) */
    { 0,      0, 65535 }, /* Mute */
};

#define TAS5733_VOLUME_MAP_SIZE ARRAY_SIZE(tas5733_volume_map)

/**
 * Convert percentage (0-100) to TAS5733L register value.
 * volume_percent_to_reg - Convert percentage (0-100) to TAS5733L register value
 * Uses linear interpolation between table points
 * @percent: Volume percentage (0-100)
 *
 * Returns: TAS5733L register value (115-65535)
 */
static unsigned int volume_percent_to_reg(unsigned int percent)
{
    int i;
    unsigned int reg_value;
    
    /* Clamp to valid range */
    if (percent > 100)
        percent = 100;
    if ((int)percent < 0)
        percent = 0;

    /* Find the two table points to interpolate between */
    for (i = 0; i < TAS5733_VOLUME_MAP_SIZE - 1; i++) {
        if (percent >= tas5733_volume_map[i + 1].percent &&
            percent <= tas5733_volume_map[i].percent) {
            
            /* Linear interpolation */
            unsigned int percent_range = tas5733_volume_map[i].percent - 
                                        tas5733_volume_map[i + 1].percent;
            unsigned int reg_range = tas5733_volume_map[i + 1].reg_value - 
                                    tas5733_volume_map[i].reg_value;
            unsigned int percent_offset = tas5733_volume_map[i].percent - percent;
            
            if (percent_range == 0)
                return tas5733_volume_map[i].reg_value;
            
            reg_value = tas5733_volume_map[i].reg_value + 
                       (reg_range * percent_offset) / percent_range;
            
            return reg_value;
        }
    }
    
    /* Default cases */
    if (percent >= 100)
        return 115;  /* Max volume */
    
    return 65535;  /* Mute */
}

/**
 * volume_reg_to_percent - Convert TAS5733L register value to percentage
 * @reg_value: TAS5733L register value
 *
 * Returns: Volume percentage (0-100)
 */
static unsigned int volume_reg_to_percent(unsigned int reg_value)
{
    int i;
    
    /* Handle mute */
    if (reg_value >= 65535)
        return 0;
    
    /* Handle max volume */
    if (reg_value <= 115)
        return 100;
    
    /* Find the two table points and interpolate */
    for (i = 0; i < TAS5733_VOLUME_MAP_SIZE - 1; i++) {
        if (reg_value >= tas5733_volume_map[i].reg_value &&
            reg_value <= tas5733_volume_map[i + 1].reg_value) {
            
            unsigned int reg_range = tas5733_volume_map[i + 1].reg_value - 
                                    tas5733_volume_map[i].reg_value;
            unsigned int percent_range = tas5733_volume_map[i].percent - 
                                        tas5733_volume_map[i + 1].percent;
            unsigned int reg_offset = reg_value - tas5733_volume_map[i].reg_value;
            
            if (reg_range == 0)
                return tas5733_volume_map[i].percent;
            
            return tas5733_volume_map[i].percent - 
                   (percent_range * reg_offset) / reg_range;
        }
    }
    
    return 0;
}



/* ============================================================================
 * Low-level Hardware Communication Functions
 * ============================================================================ */

static int fpga_spi_transfer(struct spi_device *spi, u8 *tx_buf, u8 *rx_buf, int len)
{
    struct spi_transfer xfer = {
        .tx_buf = tx_buf,
        .rx_buf = rx_buf,
        .len = len,
        .speed_hz = 1000000,  /* 1 MHz SPI clock */
        .bits_per_word = 8,
    };
    struct spi_message msg;
    int ret;

    spi_message_init(&msg);
    spi_message_add_tail(&xfer, &msg);
    
    ret = spi_sync(spi, &msg);
    if (ret < 0) {
        dev_err(&spi->dev, "SPI transfer failed: %d\n", ret);
    }
    
    return ret;
}

static int fpga_read_reg(struct spi_device *spi, u8 reg, u8 *value)
{
    u8 tx_buf[3] = {FPGA_CMD_READ, reg, 0x00};
    u8 rx_buf[3] = {0};
    int ret;

    ret = fpga_spi_transfer(spi, tx_buf, rx_buf, 3);
    if (ret == 0) {
        *value = rx_buf[2];
        dev_dbg(&spi->dev, "FPGA read reg 0x%02x = 0x%02x\n", reg, *value);
    }
    
    return ret;
}

static int fpga_write_reg(struct spi_device *spi, u8 reg, u8 value)
{
    u8 tx_buf[3] = {FPGA_CMD_WRITE, reg, value};
    int ret;

    ret = fpga_spi_transfer(spi, tx_buf, NULL, 3);
    if (ret == 0) {
        dev_dbg(&spi->dev, "FPGA write reg 0x%02x = 0x%02x\n", reg, value);
    }
    
    return ret;
}

static int pcm1748_write_reg(struct spi_device *spi, u8 reg, u8 value)
{
    u8 tx_buf[2] = {reg, value};
    int ret;

    ret = fpga_spi_transfer(spi, tx_buf, NULL, 2);
    if (ret == 0) {
        dev_dbg(&spi->dev, "PCM1748 write reg 0x%02x = 0x%02x\n", reg, value);
    }
    
    return ret;
}

static int tas5733_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
    int ret = i2c_smbus_write_byte_data(client, reg, value);
    if (ret == 0) {
        dev_dbg(&client->dev, "TAS5733 write reg 0x%02x = 0x%02x\n", reg, value);
    } else {
        dev_err(&client->dev, "TAS5733 write reg 0x%02x failed: %d\n", reg, ret);
    }
    return ret;
}

static int tas5733_read_reg(struct i2c_client *client, u8 reg, u8 *value)
{
    int ret = i2c_smbus_read_byte_data(client, reg);
    if (ret >= 0) {
        *value = ret;
        dev_dbg(&client->dev, "TAS5733 read reg 0x%02x = 0x%02x\n", reg, *value);
        return 0;
    } else {
        dev_err(&client->dev, "TAS5733 read reg 0x%02x failed: %d\n", reg, ret);
        return ret;
    }
}

static int tas5733_write_reg16(struct i2c_client *client, u8 reg, u16 value)
{
    u8 data[2] = {(value >> 8) & 0xFF, value & 0xFF};
    int ret = i2c_smbus_write_i2c_block_data(client, reg, 2, data);
    if (ret == 0) {
        dev_dbg(&client->dev, "TAS5733 write reg16 0x%02x = 0x%04x\n", reg, value);
    } else {
        dev_err(&client->dev, "TAS5733 write reg16 0x%02x failed: %d\n", reg, ret);
    }
    return ret;
}

static int tas5733_read_reg16(struct i2c_client *client, u8 reg, u16 *value)
{
    u8 data[2];
    int ret = i2c_smbus_read_i2c_block_data(client, reg, 2, data);
    if (ret == 2) {
        *value = (data[0] << 8) | data[1];
        dev_dbg(&client->dev, "TAS5733 read reg16 0x%02x = 0x%04x\n", reg, *value);
        return 0;
    } else {
        dev_err(&client->dev, "TAS5733 read reg16 0x%02x failed: %d\n", reg, ret);
        return ret < 0 ? ret : -EIO;
    }
}

/* ============================================================================
 * Hardware Initialization Functions
 * ============================================================================ */

static int tas5733_verify_device(struct i2c_client *client)
{
    u8 device_id;
    int ret;

    ret = tas5733_read_reg(client, TAS5733_DEVICE_ID, &device_id);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read device ID: %d\n", ret);
        return ret;
    }

    dev_dbg(&client->dev, "TAS5733 Device ID: 0x%02x\n", device_id);
    
    /* Check for expected device ID (adjust as needed) */
    if ((device_id & 0xFF) != 0x41) {
        dev_warn(&client->dev, "Unexpected device ID: 0x%02x\n", device_id);
    }
    
    return 0;
}

static int tas5733_init(struct densitron_audio_priv *priv)
{
    struct i2c_client *client = priv->tas5733_client;
    unsigned int init_volume_reg;
    u8 audio_ctrl_reg;
    int ret;

    dev_dbg(&client->dev, "Initializing TAS5733L power amplifier...\n");

    /* Verify device presence */
    ret = tas5733_verify_device(client);
    if (ret < 0) {
        return ret;
    }

    /* Reset and initialize according to datasheet */
    ret = tas5733_write_reg(client, TAS5733_TRIM_REG, 0x00);
    if (ret < 0) goto error;
    
    msleep(50);  /* Wait for trim to take effect */
    
    /* Configure clock for 48kHz, 24-bit Left Justified, 46.08MHz MCLK */
    ret = tas5733_write_reg(client, TAS5733_CLOCK_CTRL, 0x6C);
    if (ret < 0) goto error;
    
    /* Set modulation scheme - Bridge-tied load, BD mode */
    ret = tas5733_write_reg(client, TAS5733_MOD_SCHEME, 0x88);
    if (ret < 0) goto error;
    
    /* Configure audio format: 24-bit Left Justified */
    ret = tas5733_write_reg(client, TAS5733_AUDIO_FORMAT, 0x08);
    if (ret < 0) goto error;
    
    /* Configure current detection */
    ret = tas5733_write_reg(client, TAS5733_ICD1, 0x80);
    if (ret < 0) goto error;
    ret = tas5733_write_reg(client, TAS5733_ICD2, 0x7C);
    if (ret < 0) goto error;
    ret = tas5733_write_reg(client, TAS5733_ICD3, 0x80);
    if (ret < 0) goto error;
    ret = tas5733_write_reg(client, TAS5733_ICD4, 0x7C);
    if (ret < 0) goto error;

    /* Set initial volumes to 50% */
    init_volume_reg = volume_percent_to_reg(50);
    
    ret = tas5733_write_reg16(client, TAS5733_MASTER_VOL, 0x0100);
    if (ret < 0) goto error;
    ret = tas5733_write_reg16(client, TAS5733_LEFT_VOL, init_volume_reg);
    if (ret < 0) goto error;
    ret = tas5733_write_reg16(client, TAS5733_RIGHT_VOL, init_volume_reg);
    if (ret < 0) goto error;
    
    /* Clear error status */
    ret = tas5733_write_reg(client, TAS5733_ERROR_STATUS, 0x00);
    if (ret < 0) goto error;
    
    /* Enable ternary modulation and unmute */
    ret = tas5733_write_reg(client, TAS5733_SYS_CTRL2, 0x80);
    if (ret < 0) goto error;
    
    /* Store initial volume values */
    priv->speaker_left_vol = init_volume_reg;
    priv->speaker_right_vol = init_volume_reg;
    
    /* Enable 12V amplifier power (12V_PD is active low, so clear bit) */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_AUDIO_CTRL0, &audio_ctrl_reg);
    if (ret == 0) {
        audio_ctrl_reg &= ~FPGA_AUDIO_12V_PD;  /* Clear bit to enable power */
        ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_AUDIO_CTRL0, audio_ctrl_reg);
        if (ret == 0) {
            dev_dbg(&client->dev, "12V amplifier power enabled\n");
        }
    }
    
    
    dev_dbg(&client->dev, "TAS5733L initialized successfully\n");
    return 0;

error:
    dev_err(&client->dev, "TAS5733L initialization failed: %d\n", ret);
    tas5733_write_reg(client, TAS5733_TRIM_REG, 0x00);  /* Reset */
    return ret;
}

static int pcm1748_init(struct densitron_audio_priv *priv)
{
    struct spi_device *spi = priv->pcm1748_spi;
    int ret;

    dev_dbg(&spi->dev, "Initializing PCM1748 headphone DAC...\n");

    /* Reg 16 (0x10): Left Channel Attenuation
     * 0xD8 (216 decimal): 0.5 * (216 - 255) = -19.5 dB */
    ret = pcm1748_write_reg(spi, PCM1748_LEFT_VOLUME, 0xD8);
    if (ret < 0) goto error;
    
    /* Reg 17 (0x11): Right Channel Attenuation
     * 0xD8 (216 decimal): 0.5 * (216 - 255) = -19.5 dB */
    ret = pcm1748_write_reg(spi, PCM1748_RIGHT_VOLUME, 0xD8);
    if (ret < 0) goto error;
    
    /* Reg 18 (0x12): Soft Mute Control
     * 0x00: MUT1=0 (L unmuted), MUT2=0 (R unmuted), OVER=0 (oversampling) */
    ret = pcm1748_write_reg(spi, PCM1748_SOFT_MUTE, 0x00);
    if (ret < 0) goto error;
    
    /* Reg 19 (0x13): DAC Operation, De-emphasis, and Sample Rate
     * 0x30 = 0b00110000:
     *   - DAC1=0 (Left DAC enabled)
     *   - DAC2=0 (Right DAC enabled)
     *   - DM12=1 (De-emphasis enabled)
     *   - DMF[1:0]=01 (48 kHz sample rate) */
    ret = pcm1748_write_reg(spi, PCM1748_DAC_CTRL, 0x30);
    if (ret < 0) goto error;
    
    /* Reg 20 (0x14): Digital Filter and Audio Data Format
     * 0x05 = 0b00000101:
     *   - FMT[2:0]=101 (24-bit Left Justified)
     *   - FLT=0 (Sharp rolloff, default) */
    ret = pcm1748_write_reg(spi, PCM1748_FORMAT_CTRL, 0x05);
    if (ret < 0) goto error;
    
    /* Store initial volume values */
    priv->hp_left_vol = 0xD8;
    priv->hp_right_vol = 0xD8;
    
    /* Store initial PCM1748 register values */
    priv->pcm1748_mute = 0x00;
    priv->pcm1748_dac_ctrl = 0x30;
    priv->pcm1748_format = 0x05;
    
    dev_dbg(&spi->dev, "PCM1748 initialized: 24-bit LJ, -19.5dB, 48kHz, De-emphasis ON\n");
    return 0;

error:
    dev_err(&spi->dev, "PCM1748 initialization failed: %d\n", ret);
    return ret;
}

static int fpga_init(struct densitron_audio_priv *priv)
{
    struct spi_device *spi = priv->fpga_spi;
    u8 status, fw_version, sysctrl, default_gain_hw;
    u8 active_irq;
    int ret;
    
    dev_dbg(&spi->dev, "Initializing FPGA...\n");

    /* Read FPGA firmware version */
    ret = fpga_read_reg(spi, FPGA_REG_FW_VERSION, &fw_version);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to read FPGA firmware version: %d\n", ret);
        return ret;
    }
    dev_info(&spi->dev, "FPGA Firmware Version: %u (0x%02x)\n", fw_version, fw_version);

    /* Test FPGA communication by reading status */
    ret = fpga_read_reg(spi, FPGA_REG_STATUS, &status);
    if (ret < 0) {
        dev_err(&spi->dev, "FPGA communication test failed: %d\n", ret);
        return ret;
    }
    dev_dbg(&spi->dev, "FPGA status: 0x%02x\n", status);
    
    /* Mask all interrupts */
    ret = fpga_write_reg(spi, FPGA_REG_INT_MASK, 0xFF);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to enable interrupts\n");
        goto error;
    }


    /* Configure SYSCTRL (0x0A): Initial audio routing
     * - Enable speaker (default output)
     * - Disable headset (will be enabled on jack insertion)
     * - Enable embedded mic (default input)
     * - Disable headset mic (will be enabled on jack insertion)
     */
    sysctrl = FPGA_SYSCTRL_SPEAKER_EN | FPGA_SYSCTRL_FRONT_MIC_EN;
    
    ret = fpga_write_reg(spi, FPGA_REG_SYSCTRL, sysctrl);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to configure SYSCTRL register\n");
        goto error;
    }
    dev_dbg(&spi->dev, "SYSCTRL initialized: 0x%02x (Speaker + Embedded Mic)\n", sysctrl);
    
    
    /* Set HP detection threshold */
    ret = fpga_write_reg(spi, FPGA_REG_HP_THRESHOLD, 0x50);
    if (ret < 0) goto error;
    
    /* Set initial mic gains to 0dB (neutral) 
     * 0dB gain corresponds to register value 0x28 (40 decimal)
     */
    default_gain_hw = mic_gain_db_to_hw(MIC_GAIN_DEFAULT_DB);
    
    ret = fpga_write_reg(spi, FPGA_REG_HEADSET_MIC_GAIN, default_gain_hw);
    if (ret < 0) goto error;
    dev_dbg(&spi->dev, "Headset Mic Gain: %+ddB (hw=0x%02x)\n", 
             MIC_GAIN_DEFAULT_DB, default_gain_hw);
    
    ret = fpga_write_reg(spi, FPGA_REG_FRONT_MIC_GAIN, default_gain_hw);
    if (ret < 0) goto error;
    dev_dbg(&spi->dev, "Embedded Mic Gain: %+ddB (hw=0x%02x)\n",
             MIC_GAIN_DEFAULT_DB, default_gain_hw);
    
    /* Turn off LEDs initially */
    ret = fpga_write_reg(spi, FPGA_REG_LED1_R, 0);
    if (ret < 0) goto error;
    ret = fpga_write_reg(spi, FPGA_REG_LED1_G, 0);
    if (ret < 0) goto error;
    ret = fpga_write_reg(spi, FPGA_REG_LED1_B, 0);
    if (ret < 0) goto error;
    ret = fpga_write_reg(spi, FPGA_REG_LED2_R, 0);
    if (ret < 0) goto error;
    ret = fpga_write_reg(spi, FPGA_REG_LED2_G, 0);
    if (ret < 0) goto error;
    ret = fpga_write_reg(spi, FPGA_REG_LED2_B, 0);
    if (ret < 0) goto error;
    
    /* Store initial values */
    priv->headset_mic_gain = default_gain_hw;
    priv->front_mic_gain = default_gain_hw;
    priv->mic_source = MIC_SOURCE_FRONT;
    priv->auto_mic_switch = true;  /* Enable automatic mic switching */
    
    /* --- initilize IRQ Section --- */
    ret = fpga_read_reg(spi, FPGA_REG_INT_STATUS, &active_irq);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to read interrupt status\n");
        goto error;
    }
    dev_dbg(&spi->dev, "INT_STATUS=0x%02X\n", active_irq);
    
    /* Clear any pending interrupts by writing 0xff to status register */
    ret = fpga_write_reg(spi, FPGA_REG_INT_STATUS, 0xFF);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to clear interrupt status\n");
        goto error;
    }
    dev_dbg(&spi->dev, "Cleared all pending interrupts\n");

    /* Restore register for reading register */
    ret = fpga_write_reg(spi, FPGA_REG_INT_STATUS, 0x00);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to clear interrupt status\n");
        goto error;
    }

    /* Keep interrupts MASKED during initialization - will be unmasked later
     * after jack is created in component probe */
    ret = fpga_write_reg(spi, FPGA_REG_INT_MASK, 0xFF);  /* 0xFF = all masked */
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to mask interrupts\n");
        goto error;
    }
    
    dev_dbg(&spi->dev, "FPGA initialized successfully (24-bit LJ, interrupts MASKED until jack ready)\n");
    return 0;

error:
    dev_err(&spi->dev, "FPGA initialization failed: %d\n", ret);
    return ret;
}


static int densitron_hardware_init(struct densitron_audio_priv *priv)
{
    int ret;

    mutex_lock(&priv->hw_lock);

    if (priv->hardware_initialized) {
        mutex_unlock(&priv->hw_lock);
        return 0;
    }

    /* Reset sequence */
    if (priv->reset_gpio) {
        dev_dbg(&priv->tas5733_client->dev, "Performing hardware reset...\n");
        
        gpiod_set_value_cansleep(priv->reset_gpio, 1);  /* Assert reset */
        msleep(20);
        gpiod_set_value_cansleep(priv->reset_gpio, 0);  /* Release reset */
        msleep(100);  /* Wait for devices to stabilize */
    }

    /* Initialize hardware components in sequence */
    ret = fpga_init(priv);
    if (ret < 0) {
        dev_err(&priv->tas5733_client->dev, "FPGA init failed: %d\n", ret);
        goto error;
    }

    ret = tas5733_init(priv);
    if (ret < 0) {
        dev_err(&priv->tas5733_client->dev, "TAS5733 init failed: %d\n", ret);
        goto error;
    }

    ret = pcm1748_init(priv);
    if (ret < 0) {
        dev_err(&priv->tas5733_client->dev, "PCM1748 init failed: %d\n", ret);
        goto error;
    }

    priv->hardware_initialized = true;
    mutex_unlock(&priv->hw_lock);

    dev_info(&priv->tas5733_client->dev, "Hardware initialization completed\n");
    return 0;

error:
    mutex_unlock(&priv->hw_lock);
    return ret;
}

/* ============================================================================
 * Audio Path Control
 * ============================================================================ */

static int set_audio_path(struct densitron_audio_priv *priv, bool local)
{
    struct spi_device *spi = priv->fpga_spi;
    u8 sysctrl;
    int ret;

    mutex_lock(&priv->hw_lock);

    /* Read current SYSCTRL register */
    ret = fpga_read_reg(spi, FPGA_REG_SYSCTRL, &sysctrl);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to read SYSCTRL: %d\n", ret);
        mutex_unlock(&priv->hw_lock);
        return ret;
    }

    /* Update audio path bits (mutually exclusive) */
    if (local) {
        /* Local speaker: enable speaker, disable headset */
        sysctrl |= (FPGA_SYSCTRL_SPEAKER_EN | FPGA_SYSCTRL_FRONT_MIC_EN);
        sysctrl &= ~(FPGA_SYSCTRL_HEADSET_EN | FPGA_SYSCTRL_HEADSET_MIC_EN);
    } else {
        /* Headphones: disable speaker, enable headset */
        sysctrl &= ~(FPGA_SYSCTRL_SPEAKER_EN | FPGA_SYSCTRL_FRONT_MIC_EN);
        sysctrl |= (FPGA_SYSCTRL_HEADSET_EN | FPGA_SYSCTRL_HEADSET_MIC_EN);
    }

    ret = fpga_write_reg(spi, FPGA_REG_SYSCTRL, sysctrl);
    if (ret == 0) {
        priv->audio_path_local = local;
        dev_dbg(&spi->dev, "Audio output switched to %s (SYSCTRL=0x%02x)\n", 
                 local ? "speaker" : "headset", sysctrl);
    } else {
        dev_err(&spi->dev, "Failed to switch audio path: %d\n", ret);
    }

    mutex_unlock(&priv->hw_lock);
    return ret;
}

/* ============================================================================
 * ALSA Control Functions
 * ============================================================================ */

static int speaker_volume_info(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_info *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1;
    uinfo->value.integer.min = 0;
    uinfo->value.integer.max = 100;
    uinfo->value.integer.step = 1;
    return 0;
}

static int speaker_volume_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
    unsigned int reg_value;
    
    if (mc->reg == TAS5733_LEFT_VOL)
        reg_value = priv->speaker_left_vol;
    else
        reg_value = priv->speaker_right_vol;
    
    /* Convert register value to percentage */
    ucontrol->value.integer.value[0] = volume_reg_to_percent(reg_value);
    
    return 0;
}
static int speaker_volume_put(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
    unsigned int percent = ucontrol->value.integer.value[0];
    unsigned int reg_value;
    int ret;
    
    /* Clamp to valid range */
    if (percent > 100)
        return -EINVAL;
    
    /* Convert percentage to register value */
    reg_value = volume_percent_to_reg(percent);
    
    dev_dbg(component->dev, "Speaker volume: %u%% -> register 0x%04X (%u)\n",
            percent, reg_value, reg_value);
    
    if (mc->reg == TAS5733_LEFT_VOL) {
        if (reg_value == priv->speaker_left_vol)
            return 0;
        ret = tas5733_write_reg16(priv->tas5733_client, TAS5733_LEFT_VOL, reg_value);
        if (ret == 0)
            priv->speaker_left_vol = reg_value;
    } else {
        if (reg_value == priv->speaker_right_vol)
            return 0;
        ret = tas5733_write_reg16(priv->tas5733_client, TAS5733_RIGHT_VOL, reg_value);
        if (ret == 0)
            priv->speaker_right_vol = reg_value;
    }
    
    return ret ? ret : 1;
}

static int headphone_volume_get(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
    
    if (mc->reg == PCM1748_LEFT_VOLUME)
        ucontrol->value.integer.value[0] = priv->hp_left_vol;
    else
        ucontrol->value.integer.value[0] = priv->hp_right_vol;
        
    return 0;
}

static int headphone_volume_put(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
    unsigned int val = ucontrol->value.integer.value[0];
    int ret;
    
    if (val > 255)
        return -EINVAL;
        
    if (mc->reg == PCM1748_LEFT_VOLUME) {
        if (val == priv->hp_left_vol)
            return 0;
        ret = pcm1748_write_reg(priv->pcm1748_spi, PCM1748_LEFT_VOLUME, val);
        if (ret == 0)
            priv->hp_left_vol = val;
    } else {
        if (val == priv->hp_right_vol)
            return 0;
        ret = pcm1748_write_reg(priv->pcm1748_spi, PCM1748_RIGHT_VOLUME, val);
        if (ret == 0)
            priv->hp_right_vol = val;
    }
    
    return ret ? ret : 1;
}



static int mic_gain_get(struct snd_kcontrol *kcontrol,
                        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
    u8 hw_value;
    int gain_db;
    
    if (mc->reg == FPGA_REG_HEADSET_MIC_GAIN)
        hw_value = priv->headset_mic_gain;
    else if (mc->reg == FPGA_REG_FRONT_MIC_GAIN)
        hw_value = priv->front_mic_gain;
    else
        return -EINVAL;
    
    /* Convert hardware value to dB */
    gain_db = mic_gain_hw_to_db(hw_value);
    
    /* Report in 0.5dB units (multiply by 2) for ALSA */
    ucontrol->value.integer.value[0] = gain_db * 2;
    
    return 0;
}

static int mic_gain_put(struct snd_kcontrol *kcontrol,
                        struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    struct soc_mixer_control *mc = (struct soc_mixer_control *)kcontrol->private_value;
    int gain_db_x2 = ucontrol->value.integer.value[0];  /* In 0.5dB units */
    int gain_db = gain_db_x2 / 2;  /* Convert to integer dB */
    u8 hw_val;
    int ret;
    
    /* Convert dB to hardware register value */
    hw_val = mic_gain_db_to_hw(gain_db);
    
    if (mc->reg == FPGA_REG_HEADSET_MIC_GAIN) {
        if (hw_val == priv->headset_mic_gain)
            return 0;
        dev_dbg(component->dev, "Headset Mic Gain: %+d.%ddB (hw=0x%02x)\n",
                 gain_db, (gain_db_x2 % 2) * 5, hw_val);
        ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_HEADSET_MIC_GAIN, hw_val);
        if (ret == 0)
            priv->headset_mic_gain = hw_val;
    } else if (mc->reg == FPGA_REG_FRONT_MIC_GAIN) {
        if (hw_val == priv->front_mic_gain)
            return 0;
        dev_dbg(component->dev, "Front Mic Gain: %+d.%ddB (hw=0x%02x)\n",
                 gain_db, (gain_db_x2 % 2) * 5, hw_val);
        ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_FRONT_MIC_GAIN, hw_val);
        if (ret == 0)
            priv->front_mic_gain = hw_val;
    } else {
        return -EINVAL;
    }
    
    return ret ? ret : 1;
}

static int set_mic_source(struct densitron_audio_priv *priv, enum mic_source source)
{
    struct spi_device *spi = priv->fpga_spi;
    u8 sysctrl;
    int ret;

    mutex_lock(&priv->hw_lock);

    /* Read current SYSCTRL register */
    ret = fpga_read_reg(spi, FPGA_REG_SYSCTRL, &sysctrl);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to read SYSCTRL: %d\n", ret);
        mutex_unlock(&priv->hw_lock);
        return ret;
    }

    /* Update microphone enable bits */
    switch (source) {
    case MIC_SOURCE_NONE:
        sysctrl &= ~ FPGA_SYSCTRL_FRONT_MIC_EN;
        sysctrl &= ~FPGA_SYSCTRL_HEADSET_MIC_EN;
        dev_dbg(&spi->dev, "Microphone source: None\n");
        break;
        
    case MIC_SOURCE_FRONT:
        sysctrl |= FPGA_SYSCTRL_FRONT_MIC_EN;
        sysctrl &= ~FPGA_SYSCTRL_HEADSET_MIC_EN;
        dev_dbg(&spi->dev, "Microphone source: Front\n");
        break;
        
    case MIC_SOURCE_HEADSET:
        sysctrl &= ~FPGA_SYSCTRL_FRONT_MIC_EN;
        sysctrl |= FPGA_SYSCTRL_HEADSET_MIC_EN;
        dev_dbg(&spi->dev, "Microphone source: Headset\n");
        break;
        
    case MIC_SOURCE_BOTH:
        sysctrl |= FPGA_SYSCTRL_FRONT_MIC_EN;
        sysctrl |= FPGA_SYSCTRL_HEADSET_MIC_EN;
        dev_dbg(&spi->dev, "Microphone source: Both (Front + Headset)\n");
        break;
        
    default:
        dev_err(&spi->dev, "Invalid microphone source: %d\n", source);
        mutex_unlock(&priv->hw_lock);
        return -EINVAL;
    }

    ret = fpga_write_reg(spi, FPGA_REG_SYSCTRL, sysctrl);
    if (ret == 0) {
        priv->mic_source = source;
        dev_dbg(&spi->dev, "SYSCTRL updated: 0x%02x\n", sysctrl);
    } else {
        dev_err(&spi->dev, "Failed to set microphone source: %d\n", ret);
    }

    mutex_unlock(&priv->hw_lock);
    return ret;
}


static int audio_path_get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    
    ucontrol->value.enumerated.item[0] = priv->audio_path_local ? 0 : 1;
    return 0;
}

static int audio_path_put(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    unsigned int val = ucontrol->value.enumerated.item[0];
    bool local = (val == 0);
    int ret;
    
    if (val > 1)
        return -EINVAL;
        
    if (local == priv->audio_path_local)
        return 0;
        
    ret = set_audio_path(priv, local);
    return ret ? ret : 1;
}

/* TLV declarations */

/* ### Microphone Gain TLV
 *
 * Range: +20dB to -107.5dB in 0.5dB steps
 * Hardware inverted: 0x00 = +20dB, 0xFF = -107.5dB
 * 
 * TLV format: min_dB (in 0.01dB units), step (in 0.01dB units), mute_flag
 * +20dB = +2000 (in 0.01dB units)
 * -107.5dB = -10750 (in 0.01dB units)
 * step = 0.5dB = 50 (in 0.01dB units)
  
 * ### Speaker Gain TLV 
 *
 * Based on measured data: 115 (5W/100%) to 600 (0W/0%)
 * Approximate dB range: 0dB to -48dB (mute)
 * Scale: -48dB to 0dB in 0.48dB steps
 
      | Volume | Output Power | Register Value (base 10) | 
      | :----: | :----------: | :----------------------: | 
      | 100    |     5 W      | 115                      | 
      | 80     |     4 W      | 145                      | 
      | 60     |     3 W      | 157                      | 
      | 40     |     2 W      | 170                      | 
      | 20     |     1 W      | 195                      | 
      | 10     |     0.5 W    | 220                      | 
      | 5      |     0.1 W    | 278                      | 
      | 0      |     0 W      | 600                      |
 
 */
static const DECLARE_TLV_DB_SCALE(speaker_tlv, -4800, 48, 1);
static const DECLARE_TLV_DB_SCALE(headphone_tlv, -6350, 50, 1);
static const DECLARE_TLV_DB_SCALE(mic_gain_tlv, -10750, 50, 1);

/* Audio path enum */
static const char *audio_path_texts[] = {"Speakers", "Headphones"};
static SOC_ENUM_SINGLE_DECL(audio_path_enum, SND_SOC_NOPM, 0, audio_path_texts);

/* Microphone source enum */
static const char *mic_source_texts[] = {"None", "Front", "Headset", "Both"};
static SOC_ENUM_SINGLE_DECL(mic_source_enum, SND_SOC_NOPM, 0, mic_source_texts);

static int mic_gain_info(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_info *uinfo)
{
    uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
    uinfo->count = 1;
    /* Report range in 0.5dB steps: +20dB to -107.5dB = 255 steps */
    uinfo->value.integer.min = MIC_GAIN_MIN_DB * 2;  /* -215 (for 0.5dB resolution) */
    uinfo->value.integer.max = MIC_GAIN_MAX_DB * 2;  /* +40 (for 0.5dB resolution) */
    uinfo->value.integer.step = 1;                    /* 0.5dB steps */
    return 0;
}

static int mic_source_get(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    
    ucontrol->value.enumerated.item[0] = priv->mic_source;
    return 0;
}

static int mic_source_put(struct snd_kcontrol *kcontrol,
                          struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_component *component = snd_soc_kcontrol_component(kcontrol);
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    unsigned int val = ucontrol->value.enumerated.item[0];
    int ret;
    
    if (val >= ARRAY_SIZE(mic_source_texts))
        return -EINVAL;
        
    if (val == priv->mic_source)
        return 0;
    
    /* Disable auto-switching when manually selecting */
    priv->auto_mic_switch = false;
    
    ret = set_mic_source(priv, (enum mic_source)val);
    return ret ? ret : 1;
}


/* ALSA Control Definitions */
static const struct snd_kcontrol_new densitron_audio_controls[] = {
    /* ============ PLAYBACK CONTROLS ============ */
    
    /* Audio Output Selection */
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Audio Path Playback Route",
        .info = snd_soc_info_enum_double,
        .get = audio_path_get,
        .put = audio_path_put,
        .private_value = (unsigned long)&audio_path_enum,
    },
    
    /* Speaker Volume */
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Speaker Left Playback Volume",
        .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
        .info = speaker_volume_info,
        .get = speaker_volume_get,
        .put = speaker_volume_put,
        .private_value = SOC_SINGLE_VALUE(TAS5733_LEFT_VOL, 0, 100, 0, 0),
    },
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Speaker Right Playback Volume",
        .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
        .info = speaker_volume_info,
        .get = speaker_volume_get,
        .put = speaker_volume_put,
        .private_value = SOC_SINGLE_VALUE(TAS5733_RIGHT_VOL, 0, 100, 0, 0),
    },
    
    /* Headphone Volume */
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Headphone Left Playback Volume",
        .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
        .info = snd_soc_info_volsw,
        .get = headphone_volume_get,
        .put = headphone_volume_put,
        .private_value = SOC_SINGLE_VALUE(PCM1748_LEFT_VOLUME, 0, 255, 0, 0),
    },
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Headphone Right Playback Volume",
        .access = SNDRV_CTL_ELEM_ACCESS_READWRITE,
        .info = snd_soc_info_volsw,
        .get = headphone_volume_get,
        .put = headphone_volume_put,
        .private_value = SOC_SINGLE_VALUE(PCM1748_RIGHT_VOLUME, 0, 255, 0, 0),
    },
    
    /* ============ CAPTURE CONTROLS ============ */
    
    /* Microphone Source Selection */
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Input Source",
        .info = snd_soc_info_enum_double,
        .get = mic_source_get,
        .put = mic_source_put,
        .private_value = (unsigned long)&mic_source_enum,
    },
    
    /* Microphone Gains */
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Front Mic Capture Volume",
        .access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
                  SNDRV_CTL_ELEM_ACCESS_READWRITE,
        .tlv.p = mic_gain_tlv,
        .info = mic_gain_info,
        .get = mic_gain_get,
        .put = mic_gain_put,
        .private_value = SOC_SINGLE_VALUE(FPGA_REG_FRONT_MIC_GAIN, 0, 255, 0, 0),
    },
    {
        .iface = SNDRV_CTL_ELEM_IFACE_MIXER,
        .name = "Headset Mic Capture Volume",
        .access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |
                  SNDRV_CTL_ELEM_ACCESS_READWRITE,
        .tlv.p = mic_gain_tlv,
        .info = mic_gain_info,
        .get = mic_gain_get,
        .put = mic_gain_put,
        .private_value = SOC_SINGLE_VALUE(FPGA_REG_HEADSET_MIC_GAIN, 0, 255, 0, 0),
    },
};

/* ============================================================================
 * LED and Input Device Functions
 * ============================================================================ */

static void led_brightness_set(struct led_classdev *led_cdev,
                               enum led_brightness brightness)
{
    struct densitron_audio_priv *priv;
    u8 reg;
    
    /* Early exit if device is being removed */
    if (!led_cdev || !led_cdev->name)
        return;

    /* Determine which LED this is by checking the container */
    if (strstr(led_cdev->name, "led1")) {
        if (strstr(led_cdev->name, "red")) {
            priv = container_of(led_cdev, struct densitron_audio_priv, led1_r);
            reg = FPGA_REG_LED1_R;
        } else if (strstr(led_cdev->name, "green")) {
            priv = container_of(led_cdev, struct densitron_audio_priv, led1_g);
            reg = FPGA_REG_LED1_G;
        } else { // blue
            priv = container_of(led_cdev, struct densitron_audio_priv, led1_b);
            reg = FPGA_REG_LED1_B;
        }
    } else {
        if (strstr(led_cdev->name, "red")) {
            priv = container_of(led_cdev, struct densitron_audio_priv, led2_r);
            reg = FPGA_REG_LED2_R;
        } else if (strstr(led_cdev->name, "green")) {
            priv = container_of(led_cdev, struct densitron_audio_priv, led2_g);
            reg = FPGA_REG_LED2_G;
        } else { // blue
            priv = container_of(led_cdev, struct densitron_audio_priv, led2_b);
            reg = FPGA_REG_LED2_B;
        }
    }
    
    /* Validate pointers before use - safety check */
    if (!priv || !priv->fpga_spi) {
        pr_err("densitron-audio: LED brightness set failed - invalid SPI device\n");
        return;
    }
    
    fpga_write_reg(priv->fpga_spi, reg, brightness);
}

/* ============================================================================
 * IRQ and Work Queue Functions
 * ============================================================================ */

static void fpga_irq_work(struct work_struct *work)
{
    struct densitron_audio_priv *priv = container_of(work, 
    struct densitron_audio_priv, irq_work);
    u8 status, int_status, saved_mask, encoder0, encoder1;
    bool hp_connected;
    int ret;
      
    atomic_inc(&priv->irq_count);
    
    /* Step 0: Save current interrupt mask and disable all interrupts
     * Note: 0x0E is an INTERRUPT MASK register where 1=masked (disabled), 0=unmasked (enabled)
     */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_MASK, &saved_mask);
    if (ret < 0) {
        dev_err(&priv->fpga_spi->dev, "Failed to read FPGA interrupt mask: %d\n", ret);
        return;
    }
    dev_dbg(&priv->fpga_spi->dev, "Current INT_MASK=0x%02x\n", saved_mask);
    
    /* Mask (disable) all interrupts */
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, 0xFF);
    if (ret < 0) {
        dev_err(&priv->fpga_spi->dev, "Failed to mask all interrupts: %d\n", ret);
        return;
    }
    
    
    /* Step 1: Read interrupt status to see what triggered the IRQ */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, &int_status);
    if (ret < 0) {
        /* restore irq mask */
        fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, saved_mask);
        dev_err(&priv->fpga_spi->dev, "Failed to read interrupt status: %d\n", ret);
        return;
    }
    
    dev_dbg(&priv->fpga_spi->dev, "IRQ #%d: INT_STATUS=0x%02x\n", 
             atomic_read(&priv->irq_count), int_status);
    
    /* If no interrupts are pending, something is wrong */
    if (int_status == 0) {
        fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, saved_mask);
        dev_warn(&priv->fpga_spi->dev, "Spurious interrupt - no INT_STATUS bits set\n");
        return;
    }
    
    /* Step 2: Read current signal status */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_STATUS, &status);
    if (ret < 0) {
        /* restore irq mask */
        fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, saved_mask);
        dev_err(&priv->fpga_spi->dev, "Failed to read STATUS register: %d\n", ret);
        return;
    }
    
    dev_dbg(&priv->fpga_spi->dev, "STATUS=0x%02x\n", status);
    
    /* Step 3: Handle each interrupt type and read associated data BEFORE clearing */
    
    /* Handle headphone detection interrupt */
    if (int_status & FPGA_INT_HP_DETECT) {
        hp_connected = !!(status & FPGA_STATUS_HP_DETECT);
        
        dev_info(&priv->fpga_spi->dev, 
                 "HP interrupt: jack %s (status bit=%d)\n",
                 hp_connected ? "INSERTED" : "REMOVED",
                 !!(status & FPGA_STATUS_HP_DETECT));
        
        if (hp_connected != priv->headphones_connected) {
            priv->headphones_connected = hp_connected;
            
            /* Report jack status to ALSA - only if jack is initialized */
            if (priv->component && priv->component->card && priv->headphone_jack.jack) {
                snd_soc_jack_report(&priv->headphone_jack, 
                            hp_connected ? SND_JACK_HEADPHONE : 0,
                            SND_JACK_HEADPHONE);
            }
            
            /* Automatic audio path switching */
            ret = set_audio_path(priv, !hp_connected);
            if (ret < 0) {
                dev_err(&priv->fpga_spi->dev, 
                        "Failed to switch audio path: %d\n", ret);
            }
            
            /* Automatic microphone switching (if enabled) */
            if (priv->auto_mic_switch) {
                enum mic_source new_source = hp_connected ? 
                                             MIC_SOURCE_HEADSET : 
                                             MIC_SOURCE_FRONT;
                ret = set_mic_source(priv, new_source);
                if (ret < 0) {
                    dev_err(&priv->fpga_spi->dev, 
                            "Failed to switch microphone source: %d\n", ret);
                }
            }
        }
    }
    
    /* Handle rotary encoder interrupts - READ FIRST, then clear */
    if (int_status & (FPGA_INT_ENCODER0 | FPGA_INT_ENCODER1)) {
        
        if (int_status & FPGA_INT_ENCODER1) {
            ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_ENCODER1, &encoder1);
            if (ret == 0) {
                int8_t delta = (int8_t)encoder1;

                // msleep(100); // wait for rotary encoder to stop
                
                if (delta != 0) {
                    /* Accumulate delta */
                    priv->left_rotary_position += delta;
                    
                    dev_dbg(&priv->fpga_spi->dev, 
                             "Left encoder: delta=%d, position=%d\n", 
                             delta, priv->left_rotary_position);
                    
                    /* Reset FPGA register by writing 0 */
                    fpga_write_reg(priv->fpga_spi, FPGA_REG_ENCODER1, 0);

                    /* Reset FPGA rotary encoder */
                    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, &status);
                    if (ret == 0) {                        
                        fpga_write_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, (status | FPGA_SYSCTRL_ENC1_RST));
                        fpga_write_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, status );
                    }
                    
                    /* Still report to input subsystem */
                    if (priv->left_rotary) {
                        input_report_rel(priv->left_rotary, REL_WHEEL, delta);
                        input_sync(priv->left_rotary);
                    }
                }
            }
        }
        
        if (int_status & FPGA_INT_ENCODER0) {
            ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_ENCODER0, &encoder0);
            if (ret == 0) {
                int8_t delta = (int8_t)encoder0;

                // msleep(100); // wait for rotary encoder to stop
                
                if (delta != 0) {
                    /* Accumulate delta */
                    priv->right_rotary_position += delta;
                    
                    dev_dbg(&priv->fpga_spi->dev, 
                             "Right encoder: delta=%d, position=%d\n", 
                             delta, priv->right_rotary_position);
                    
                    /* Reset FPGA register by writing 0 */
                    fpga_write_reg(priv->fpga_spi, FPGA_REG_ENCODER0, 0);

                    /* Reset FPGA rotary encoder */
                    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, &status);
                    if (ret == 0) {                        
                        fpga_write_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, (status | FPGA_SYSCTRL_ENC0_RST));
                        fpga_write_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, status );
                    }
                    
                    /* Still report to input subsystem */
                    if (priv->right_rotary) {
                        input_report_rel(priv->right_rotary, REL_WHEEL, delta);
                        input_sync(priv->right_rotary);
                    }
                }
            }
        }
    }
        
    /* Handle button press interrupts */
    if (int_status & FPGA_INT_BUTTON0) {
        ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_STATUS, &status);
        if (ret == 0) {
            /* Buttons are active low - pressed when bit is 0 */
            bool button_pressed = !(status & FPGA_STATUS_BUTTON0);
            
            dev_dbg(&priv->fpga_spi->dev, "Button 0 (Right): %s\n", 
                     button_pressed ? "PRESSED" : "RELEASED");
            
            /* Report to right rotary input device */
            if (priv->right_rotary) {
                input_report_key(priv->right_rotary, BTN_0, button_pressed);
                input_sync(priv->right_rotary);
            }
        }
    }

    if (int_status & FPGA_INT_BUTTON1) {
        ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_STATUS, &status);
        if (ret == 0) {
            /* Buttons are active low - pressed when bit is 0 */
            bool button_pressed = !(status & FPGA_STATUS_BUTTON1);
            
            dev_dbg(&priv->fpga_spi->dev, "Button 1 (Left): %s\n",
                     button_pressed ? "PRESSED" : "RELEASED");
            
            /* Report to left rotary input device */
            if (priv->left_rotary) {
                input_report_key(priv->left_rotary, BTN_0, button_pressed);
                input_sync(priv->left_rotary);
            }
        }
    }
    
    /* Step 4: Clear interrupts using write-1-to-clear mechanism
     * Write back the same value we read to clear those specific bits
     */
    dev_dbg(&priv->fpga_spi->dev, "Clearing interrupts: writing 0x%02x to INT_STATUS\n", 
             int_status);
    
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, 0xFF);
    if (ret < 0) {
        /* restore irq mask */
        fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, saved_mask);
        dev_err(&priv->fpga_spi->dev, "Failed to reset interrupts: %d\n", ret);
        return;
    }
    
    /* Reset to read */
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, 0x00); /* ToDo: not clear CPLD cleaning procedure */
    if (ret < 0) {
        /* restore irq mask */
        fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, saved_mask);
        dev_err(&priv->fpga_spi->dev, "Failed to clear interrupts: %d\n", ret);
        return;
    }
    
    /*  Re-enable interrupts (unmask) */
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, saved_mask);
    if (ret < 0) {
        dev_err(&priv->fpga_spi->dev, "Failed to applay mask to interrupts: %d\n", ret);
        return;
    }
}

static irqreturn_t fpga_irq_handler(int irq, void *dev_id)
{
    struct densitron_audio_priv *priv = dev_id;
    
    /* Schedule work to handle the interrupt */
    schedule_work(&priv->irq_work);
    
    return IRQ_HANDLED;
}

/* ============================================================================
 * ALSA SoC DAI Functions
 * ============================================================================ */

static int inf1000_set_dai_sysclk(struct snd_soc_dai *dai, int clk_id,
                                   unsigned int freq, int dir)
{
    struct snd_soc_component *component = dai->component;
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);

    dev_dbg(component->dev, "set_dai_sysclk: clk_id=%d, freq=%u, dir=%d\n", clk_id, freq, dir);

    if (dir != SND_SOC_CLOCK_IN) {
        dev_err(component->dev, "Only input MCLK supported\n");
        return -EINVAL;
    }

    /* Handle the sysclk reconfiguration issue */
    if (freq == 0) {
        /* Simple-audio-card is miscalculating - keep existing MCLK */
        if (priv->mclk == DENSITRON_MCLK_FREQ) {
            dev_dbg(component->dev, "Ignoring 0 Hz sysclk, keeping existing MCLK: %u Hz\n", priv->mclk);
            return 0;  /* Success - keep existing */
        } else {
            dev_warn(component->dev, "MCLK frequency is 0 Hz and no valid MCLK set - using default\n");
            priv->mclk = DENSITRON_MCLK_FREQ;
            return 0;
        }
    }

    /* Validate frequency */
    if (freq != DENSITRON_MCLK_FREQ) {
        dev_warn(component->dev, "MCLK is %u Hz, expected %d Hz - accepting anyway\n", 
                 freq, DENSITRON_MCLK_FREQ);
    }

    priv->mclk = freq;
    dev_dbg(component->dev, "MCLK set to %u Hz\n", freq);
    
    return 0;
}

static int inf1000_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
    struct snd_soc_component *component = codec_dai->component;
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    unsigned int format = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
    unsigned int master = fmt & SND_SOC_DAIFMT_MASTER_MASK;
    
    dev_dbg(component->dev, "set_dai_fmt: 0x%08x\n", fmt);
    
    /* STRICT: Only accept Left Justified format */
    if (format != SND_SOC_DAIFMT_LEFT_J) {
        dev_err(component->dev, "REJECTED: Only Left Justified format supported, got 0x%x\n", 
                format);
        atomic_inc(&priv->format_rejects);
        return -EINVAL;
    }
    
    /* We expect codec to be in slave mode (CPU is master) */
    if (master != SND_SOC_DAIFMT_CBS_CFS) {
        dev_err(component->dev, "REJECTED: Codec must be in slave mode, got 0x%x\n", master);
        return -EINVAL;
    }
    
    dev_dbg(component->dev, "DAI format ACCEPTED: Left Justified, Codec Slave Mode\n");
    return 0;
}

static int inf1000_pcm_startup(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
    struct snd_soc_component *component = dai->component;
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    struct snd_pcm_runtime *runtime = substream->runtime;
    int ret;

    dev_dbg(component->dev, "PCM startup: %s stream\n", 
             substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture");

    /* Ensure hardware is initialized */
    ret = densitron_hardware_init(priv);
    if (ret < 0) {
        dev_err(component->dev, "Hardware initialization failed: %d\n", ret);
        return ret;
    }

    /* CRITICAL: Set hardware constraints BEFORE any other constraints */
    runtime->hw.info = SNDRV_PCM_INFO_MMAP |
                       SNDRV_PCM_INFO_MMAP_VALID |
                       SNDRV_PCM_INFO_INTERLEAVED |
                       SNDRV_PCM_INFO_BLOCK_TRANSFER;
    
    runtime->hw.formats = SNDRV_PCM_FMTBIT_S24_LE;  /* ONLY S24_LE */
    runtime->hw.rates = SNDRV_PCM_RATE_8000_48000;
    runtime->hw.rate_min = DENSITRON_MIN_RATE;
    runtime->hw.rate_max = DENSITRON_MAX_RATE;
    runtime->hw.channels_min = DENSITRON_CHANNELS;
    runtime->hw.channels_max = DENSITRON_CHANNELS;
    runtime->hw.buffer_bytes_max = 131072;
    runtime->hw.period_bytes_min = 64;
    runtime->hw.period_bytes_max = 16384;
    runtime->hw.periods_min = 2;
    runtime->hw.periods_max = 128;
    
    /* STRICT FORMAT CONSTRAINT: Only S24_LE - no exceptions */
    ret = snd_pcm_hw_constraint_mask64(runtime, SNDRV_PCM_HW_PARAM_FORMAT,
                                       SNDRV_PCM_FMTBIT_S24_LE);
    if (ret < 0) {
        dev_err(component->dev, "Failed to set S24_LE format constraint: %d\n", ret);
        return ret;
    }

    /* STRICT RATE CONSTRAINT: Only supported rates */
    ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                     &densitron_rate_constraints);
    if (ret < 0) {
        dev_err(component->dev, "Failed to set rate constraints: %d\n", ret);
        return ret;
    }

    /* STRICT CHANNEL CONSTRAINT: Only stereo */
    ret = snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_CHANNELS, 
                                       DENSITRON_CHANNELS, DENSITRON_CHANNELS);
    if (ret < 0) {
        dev_err(component->dev, "Failed to set stereo constraint: %d\n", ret);
        return ret;
    }

    /* STRICT: No automatic format conversion */
    ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (ret < 0) {
        dev_err(component->dev, "Failed to set period constraint: %d\n", ret);
        return ret;
    }

    /* Ensure proper alignment for 24-bit samples */
    ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 4);
    if (ret < 0) {
        dev_err(component->dev, "Failed to set period alignment: %d\n", ret);
        return ret;
    }

    ret = snd_pcm_hw_constraint_step(runtime, 0, SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 4);
    if (ret < 0) {
        dev_err(component->dev, "Failed to set buffer alignment: %d\n", ret);
        return ret;
    }

    /* Force specific period/buffer size relationships for stable operation */
    ret = snd_pcm_hw_constraint_minmax(runtime, SNDRV_PCM_HW_PARAM_BUFFER_SIZE, 256, 131072);
    if (ret < 0) {
        dev_err(component->dev, "Failed to set buffer size constraint: %d\n", ret);
        return ret;
    }

    dev_dbg(component->dev, "PCM constraints applied: S24_LE ONLY, %d channels, rates %d-%d Hz\n",
             DENSITRON_CHANNELS, DENSITRON_MIN_RATE, DENSITRON_MAX_RATE);
    
    return 0;
}

static int inf1000_pcm_hw_params(struct snd_pcm_substream *substream,
                                  struct snd_pcm_hw_params *params,
                                  struct snd_soc_dai *dai)
{
    struct snd_soc_component *component = dai->component;
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    unsigned int rate = params_rate(params);
    unsigned int channels = params_channels(params);
    unsigned int width = params_width(params);
    unsigned int physical_width = params_physical_width(params);
    snd_pcm_format_t format = params_format(params);
    unsigned int mclk_ratio;

    dev_dbg(component->dev, "hw_params: rate=%u, channels=%u, width=%u, phys_width=%u, format=%d\n", 
             rate, channels, width, physical_width, format);

    /* FORCE correct MCLK if it's wrong */
    if (priv->mclk != DENSITRON_MCLK_FREQ) {
        dev_warn(component->dev, "Correcting MCLK from %u to %d Hz\n", priv->mclk, DENSITRON_MCLK_FREQ);
        priv->mclk = DENSITRON_MCLK_FREQ;
    }

    /* STRICT VALIDATION */
    if (format != DENSITRON_FORMAT) {
        dev_err(component->dev, "REJECTED: Format must be S24_LE, got %d\n", format);
        atomic_inc(&priv->format_rejects);
        return -EINVAL;
    }

    if (channels != DENSITRON_CHANNELS) {
        dev_err(component->dev, "REJECTED: Must be %d channels, got %u\n", 
                DENSITRON_CHANNELS, channels);
        return -EINVAL;
    }

    if (rate < DENSITRON_MIN_RATE || rate > DENSITRON_MAX_RATE) {
        dev_err(component->dev, "REJECTED: Rate %u not in range %d-%d\n", 
                rate, DENSITRON_MIN_RATE, DENSITRON_MAX_RATE);
        return -EINVAL;
    }

    /* Calculate and validate MCLK ratio */
    mclk_ratio = priv->mclk / rate;
    if (priv->mclk % rate != 0) {
        dev_err(component->dev, "REJECTED: MCLK %u not multiple of rate %u\n", 
                priv->mclk, rate);
        return -EINVAL;
    }
    
    dev_dbg(component->dev, "MCLK ratio: %u (%u Hz / %u Hz)\n", 
             mclk_ratio, priv->mclk, rate);

    /* Store current parameters */
    priv->sample_rate = rate;
    
    dev_dbg(component->dev, "ACCEPTED: %u Hz, %d channels, S24_LE (%d-bit)\n", 
             rate, channels, width);
    
    return 0;
}


static int inf1000_mute(struct snd_soc_dai *dai, int mute, int direction)
{
    struct snd_soc_component *component = dai->component;
    
    /* v4.6 fix: Cannot do I2C/SPI operations here - called from atomic context
     * (trigger callback with spinlocks held). Hardware mute is optional and not
     * strictly necessary for basic functionality. If needed, it could be moved to
     * prepare/hw_params callbacks which run in process context. */
    
    dev_dbg(component->dev, "Mute %s: %s (skipped - atomic context)\n", 
             direction == SNDRV_PCM_STREAM_PLAYBACK ? "playback" : "capture",
             mute ? "ON" : "OFF");
    
    return 0;
}

static const struct snd_soc_dai_ops inf1000_dai_ops = {
    .startup = inf1000_pcm_startup,
    .hw_params = inf1000_pcm_hw_params,
    .mute_stream = inf1000_mute,
    .set_fmt = inf1000_set_dai_fmt,
    .set_sysclk = inf1000_set_dai_sysclk,
    .no_capture_mute = 1,
};

static struct snd_soc_dai_driver inf1000_dai = {
    .name = "densitron-audio-hifi",
    .playback = {
        .stream_name = "Playback",
        .channels_min = DENSITRON_CHANNELS,
        .channels_max = DENSITRON_CHANNELS,
        .rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
                 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
        .formats = SNDRV_PCM_FMTBIT_S24_LE,  /* ONLY S24_LE */
    },
    .capture = {
        .stream_name = "Capture", 
        .channels_min = DENSITRON_CHANNELS,
        .channels_max = DENSITRON_CHANNELS,
        .rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 |
                 SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000,
        .formats = SNDRV_PCM_FMTBIT_S24_LE,  /* ONLY S24_LE */
    },
    .ops = &inf1000_dai_ops,
    .symmetric_rate = 1,
    .symmetric_channels = 1,
    .symmetric_sample_bits = 1,
};

/* ============================================================================
 * ALSA SoC Component Functions
 * ============================================================================ */

static int densitron_audio_component_probe(struct snd_soc_component *component)
{
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    u8 status;
    u8 int_enable = FPGA_INT_MSK_HP_DETECT |    /* Headphone jack detection */
                FPGA_INT_MSK_BUTTON0 |       /* Button 0 */
                FPGA_INT_MSK_BUTTON1 |       /* Button 1 */
                FPGA_INT_MSK_ENCODER0 |      /* Rotary encoder 0 */
                FPGA_INT_MSK_ENCODER1;       /* Rotary encoder 1 */
    int ret;

    priv->component = component;

    dev_dbg(component->dev, "Densitron audio component probe started\n");

    /* Initialize hardware */
    ret = densitron_hardware_init(priv);
    if (ret < 0) {
        dev_err(component->dev, "Hardware initialization failed: %d\n", ret);
        return ret;
    }
    
    /* Initialize jack detection */
    ret = snd_soc_card_jack_new(component->card, "Headphones",
                                SND_JACK_HEADPHONE, &priv->headphone_jack,
                                NULL, 0);
    if (ret < 0) {
        dev_err(component->dev, "Failed to create headphone jack: %d\n", ret);
        return ret;
    }

    /* Now jack is ready: enable FPGA interrupts (disable masks) - v4.6 fix */
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, ~int_enable);
    if (ret < 0) {
        dev_warn(component->dev, "Failed to unmask FPGA interrupts: %d\n", ret);
        /* Non-fatal, continue */
    } else {
        dev_dbg(component->dev, "FPGA interrupts enabled: 0x%02x\n", int_enable);
    }
    
    /* Read and report initial headphone state */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_STATUS, &status);
    if (ret == 0) {
        priv->headphones_connected = !!(status & FPGA_STATUS_HP_DETECT);
        if (priv->headphones_connected) {
            dev_dbg(component->dev, "Headphone detected at initialization\n");
            snd_soc_jack_report(&priv->headphone_jack, 
                              SND_JACK_HEADPHONE, SND_JACK_HEADPHONE);
            /* Switch to headphone output */
            ret = set_audio_path(priv, false);
            if (ret < 0) {
                dev_err(component->dev, "Failed to set audio path to headphones: %d\n", ret);
            }
        } else {
            /* No headphone detected, use speakers */
            ret = set_audio_path(priv, true);
            if (ret < 0) {
                dev_err(component->dev, "Failed to set audio path to speakers: %d\n", ret);
            }
        }
    } else {
    /* Couldn't read status, default to speakers */
    ret = set_audio_path(priv, true);
    if (ret < 0) {
        dev_err(component->dev, "Failed to set default audio path: %d\n", ret);
        return ret;
        }
    }

    dev_info(component->dev, "Densitron audio component probe completed successfully\n");
    return 0;
}

static void densitron_audio_component_remove(struct snd_soc_component *component)
{
    struct densitron_audio_priv *priv = snd_soc_component_get_drvdata(component);
    
    dev_info(component->dev, "Densitron audio component remove\n");
    
    /* Mask all FPGA interrupts before cleanup - v4.6 fix */
    if (priv->hardware_initialized) {
        fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, 0xFF);
    }

    /* Cancel any pending work */
    cancel_work_sync(&priv->irq_work);
    
    /* Mute all outputs */
    if (priv->hardware_initialized) {
        tas5733_write_reg(priv->tas5733_client, TAS5733_SOFT_MUTE, 0x07);
        pcm1748_write_reg(priv->pcm1748_spi, PCM1748_SOFT_MUTE, 0x01);
    }
}


static const struct snd_soc_component_driver densitron_audio_component = {
    .probe = densitron_audio_component_probe,
    .remove = densitron_audio_component_remove,
    .controls = densitron_audio_controls,
    .num_controls = ARRAY_SIZE(densitron_audio_controls),
    .suspend_bias_off = 1,
    .idle_bias_on = 0,
    .use_pmdown_time = 1,
    .endianness = 1,
    .non_legacy_dai_naming = 1,
};

/* ============================================================================
 * Input and LED Device Setup
 * ============================================================================ */
static int setup_input_devices(struct densitron_audio_priv *priv)
{
    struct device *dev = &priv->fpga_spi->dev;
    int ret;

    /* Left rotary encoder */
    priv->left_rotary = input_allocate_device();
    if (!priv->left_rotary)
        return -ENOMEM;

    priv->left_rotary->name = "Densitron Left Rotary";
    priv->left_rotary->phys = "densitron/input/rotary0";
    priv->left_rotary->id.bustype = BUS_HOST;
    priv->left_rotary->id.vendor = 0x0001;
    priv->left_rotary->id.product = 0x0001;
    priv->left_rotary->id.version = 0x0100;
    priv->left_rotary->dev.parent = dev;
    
    /* Set capabilities */
    __set_bit(EV_REL, priv->left_rotary->evbit);
    __set_bit(REL_WHEEL, priv->left_rotary->relbit);
    __set_bit(EV_KEY, priv->left_rotary->evbit);
    __set_bit(BTN_0, priv->left_rotary->keybit);
    
    ret = input_register_device(priv->left_rotary);
    if (ret < 0) {
        dev_err(dev, "Failed to register left rotary input: %d\n", ret);
	input_free_device(priv->left_rotary);
        return ret;
    }

    /* Right rotary encoder */
    priv->right_rotary = input_allocate_device();
    if (!priv->right_rotary)
        return -ENOMEM;

    priv->right_rotary->name = "Densitron Right Rotary";
    priv->right_rotary->phys = "densitron/input/rotary1";
    priv->right_rotary->id.bustype = BUS_HOST;
    priv->right_rotary->id.vendor = 0x0001;
    priv->right_rotary->id.product = 0x0002;
    priv->right_rotary->id.version = 0x0100;
    priv->right_rotary->dev.parent = dev;
    
    /* Set capabilities */
    __set_bit(EV_REL, priv->right_rotary->evbit);
    __set_bit(REL_WHEEL, priv->right_rotary->relbit);
    __set_bit(EV_KEY, priv->right_rotary->evbit);
    __set_bit(BTN_0, priv->right_rotary->keybit);
    
    ret = input_register_device(priv->right_rotary);
    if (ret < 0) {
        dev_err(dev, "Failed to register right rotary input: %d\n", ret);
	input_free_device(priv->right_rotary);
        return ret;
    }

    dev_info(dev, "Input devices registered successfully\n");
    return 0;
}

static int setup_led_devices(struct densitron_audio_priv *priv)
{
    struct device *dev = &priv->fpga_spi->dev;
    int ret;

    /* LED 1 RGB */
    priv->led1_r.name = "densitron:red:led1";
    priv->led1_r.brightness_set = led_brightness_set;
    priv->led1_r.max_brightness = 255;
    ret = devm_led_classdev_register(dev, &priv->led1_r);
    if (ret < 0) {
        dev_err(dev, "Failed to register LED1 red: %d\n", ret);
        return ret;
    }

    priv->led1_g.name = "densitron:green:led1";
    priv->led1_g.brightness_set = led_brightness_set;
    priv->led1_g.max_brightness = 255;
    ret = devm_led_classdev_register(dev, &priv->led1_g);
    if (ret < 0) {
        dev_err(dev, "Failed to register LED1 green: %d\n", ret);
        return ret;
    }

    priv->led1_b.name = "densitron:blue:led1";
    priv->led1_b.brightness_set = led_brightness_set;
    priv->led1_b.max_brightness = 255;
    ret = devm_led_classdev_register(dev, &priv->led1_b);
    if (ret < 0) {
        dev_err(dev, "Failed to register LED1 blue: %d\n", ret);
        return ret;
    }

    /* LED 2 RGB */
    priv->led2_r.name = "densitron:red:led2";
    priv->led2_r.brightness_set = led_brightness_set;
    priv->led2_r.max_brightness = 255;
    ret = devm_led_classdev_register(dev, &priv->led2_r);
    if (ret < 0) {
        dev_err(dev, "Failed to register LED2 red: %d\n", ret);
        return ret;
    }

    priv->led2_g.name = "densitron:green:led2";
    priv->led2_g.brightness_set = led_brightness_set;
    priv->led2_g.max_brightness = 255;
    ret = devm_led_classdev_register(dev, &priv->led2_g);
    if (ret < 0) {
        dev_err(dev, "Failed to register LED2 green: %d\n", ret);
        return ret;
    }

    priv->led2_b.name = "densitron:blue:led2";
    priv->led2_b.brightness_set = led_brightness_set;
    priv->led2_b.max_brightness = 255;
    ret = devm_led_classdev_register(dev, &priv->led2_b);
    if (ret < 0) {
        dev_err(dev, "Failed to register LED2 blue: %d\n", ret);
        return ret;
    }

    dev_info(dev, "LED devices registered successfully\n");
    return 0;
}

/* ============================================================================
 * Debug sysfs Interface for FPGA Register Access
 * ============================================================================ */

static ssize_t fpga_status_show(struct device *dev,
                                 struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 status;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_STATUS, &status);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (bin: %c%c%c%c%c%c%c%c)\n  bit4(HP)=%d bit2(BTN1)=%d bit0(BTN0)=%d\n",
                   status,
                   (status & BIT(7)) ? '1' : '0',
                   (status & BIT(6)) ? '1' : '0',
                   (status & BIT(5)) ? '1' : '0',
                   (status & BIT(4)) ? '1' : '0',
                   (status & BIT(3)) ? '1' : '0',
                   (status & BIT(2)) ? '1' : '0',
                   (status & BIT(1)) ? '1' : '0',
                   (status & BIT(0)) ? '1' : '0',
                   !!(status & FPGA_STATUS_HP_DETECT),
                   !!(status & FPGA_STATUS_BUTTON1),
                   !!(status & FPGA_STATUS_BUTTON0));
}
static DEVICE_ATTR_RO(fpga_status);

static ssize_t fpga_int_status_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 int_status;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, &int_status);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (bin: %c%c%c%c%c%c%c%c)\n  HP=%d ENC1=%d BTN1=%d ENC0=%d BTN0=%d\n",
                   int_status,
                   (int_status & BIT(7)) ? '1' : '0',
                   (int_status & BIT(6)) ? '1' : '0',
                   (int_status & BIT(5)) ? '1' : '0',
                   (int_status & BIT(4)) ? '1' : '0',
                   (int_status & BIT(3)) ? '1' : '0',
                   (int_status & BIT(2)) ? '1' : '0',
                   (int_status & BIT(1)) ? '1' : '0',
                   (int_status & BIT(0)) ? '1' : '0',
                   !!(int_status & FPGA_INT_HP_DETECT),
                   !!(int_status & FPGA_INT_ENCODER1),
                   !!(int_status & FPGA_INT_BUTTON1),
                   !!(int_status & FPGA_INT_ENCODER0),
                   !!(int_status & FPGA_INT_BUTTON0));
}

static ssize_t fpga_int_status_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    u8 int_status_before, int_status_after;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    /* Read current interrupt status before clearing */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, &int_status_before);
    if (ret < 0) {
        dev_err(dev, "Failed to read INT_STATUS: %d\n", ret);
        return ret;
    }
    
    dev_dbg(dev, "INT_STATUS before: 0x%02x\n", int_status_before);
    
    /* Write the user-specified value to INT_STATUS register
     * (write-1-to-clear mechanism)
     */
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, (u8)val);
    if (ret < 0) {
        dev_err(dev, "Failed to write INT_STATUS: %d\n", ret);
        return ret;
    }
    
    dev_dbg(dev, "Wrote 0x%02x to INT_STATUS register\n", (u8)val);
    
    /* Read back to verify */
    msleep(1);
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, &int_status_after);
    if (ret < 0) {
        dev_err(dev, "Failed to read INT_STATUS after write: %d\n", ret);
        return ret;
    }
    
    dev_dbg(dev, "INT_STATUS after:  0x%02x\n", int_status_after);
    
    /* Report what changed */
    if (int_status_after == 0x00) {
        dev_dbg(dev, "All interrupts cleared successfully\n");
    } else if (int_status_after != int_status_before) {
        u8 cleared = int_status_before & ~int_status_after;
        dev_dbg(dev, "Cleared bits: 0x%02x (before: 0x%02x, after: 0x%02x)\n",
                 cleared, int_status_before, int_status_after);
    } else {
        dev_warn(dev, "INT_STATUS unchanged (still 0x%02x)\n", int_status_after);
    }
    
    return count;
}
static DEVICE_ATTR_RW(fpga_int_status);

static ssize_t fpga_int_enable_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 int_masked, int_enable;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_MASK, &int_masked);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    /* adapt to Interrupt mask in FPGA */
    int_enable = ~int_masked;
    return scnprintf(buf, PAGE_SIZE, "0x%02x (bin: %c%c%c%c%c%c%c%c)\n  HP_EN=%d ENC1_EN=%d BTN1_EN=%d ENC0_EN=%d BTN0_EN=%d\n",
                   int_enable,
                   (int_enable & BIT(7)) ? '1' : '0',
                   (int_enable & BIT(6)) ? '1' : '0',
                   (int_enable & BIT(5)) ? '1' : '0',
                   (int_enable & BIT(4)) ? '1' : '0',
                   (int_enable & BIT(3)) ? '1' : '0',
                   (int_enable & BIT(2)) ? '1' : '0',
                   (int_enable & BIT(1)) ? '1' : '0',
                   (int_enable & BIT(0)) ? '1' : '0',
                   !!(int_enable & FPGA_INT_MSK_HP_DETECT),
                   !!(int_enable & FPGA_INT_MSK_ENCODER1),
                   !!(int_enable & FPGA_INT_MSK_BUTTON1),
                   !!(int_enable & FPGA_INT_MSK_ENCODER0),
                   !!(int_enable & FPGA_INT_MSK_BUTTON0));
}

static ssize_t fpga_int_enable_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, (u8)~val);
    if (ret < 0)
        return ret;
    
    dev_dbg(dev, "Interrupt enable set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(fpga_int_enable);
static ssize_t fpga_sysctrl_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 sysctrl;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, &sysctrl);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (bin: %c%c%c%c%c%c%c%c)\n"
                   "  EMB_MIC=%d HEADSET_MIC=%d HEADSET=%d SPEAKER=%d SAI_LOOP=%d\n",
                   sysctrl,
                   (sysctrl & BIT(7)) ? '1' : '0',
                   (sysctrl & BIT(6)) ? '1' : '0',
                   (sysctrl & BIT(5)) ? '1' : '0',
                   (sysctrl & BIT(4)) ? '1' : '0',
                   (sysctrl & BIT(3)) ? '1' : '0',
                   (sysctrl & BIT(2)) ? '1' : '0',
                   (sysctrl & BIT(1)) ? '1' : '0',
                   (sysctrl & BIT(0)) ? '1' : '0',
                   !!(sysctrl & FPGA_SYSCTRL_FRONT_MIC_EN),
                   !!(sysctrl & FPGA_SYSCTRL_HEADSET_MIC_EN),
                   !!(sysctrl & FPGA_SYSCTRL_HEADSET_EN),
                   !!(sysctrl & FPGA_SYSCTRL_SPEAKER_EN),
                   !!(sysctrl & FPGA_SYSCTRL_SAI_LOOP));
}

static ssize_t fpga_sysctrl_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, (u8)val);
    if (ret < 0)
        return ret;
    
    dev_dbg(dev, "SYSCTRL set to 0x%02x\n", (u8)val);
    return count;
}

static DEVICE_ATTR_RW(fpga_sysctrl);

static ssize_t fpga_fw_version_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 fw_version;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_FW_VERSION, &fw_version);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "%u (0x%02x)\n", fw_version, fw_version);
}
static DEVICE_ATTR_RO(fpga_fw_version);

/* ============================================================================
 * FPGA Audio Configuration Registers SYSFS Interface
 * ============================================================================ */

static ssize_t fpga_headset_mic_gain_show(struct device *dev,
                                           struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 gain;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_HEADSET_MIC_GAIN, &gain);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (%u) = %+d dB\n", 
                   gain, gain, mic_gain_hw_to_db(gain));
}

static ssize_t fpga_headset_mic_gain_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_HEADSET_MIC_GAIN, (u8)val);
    if (ret < 0)
        return ret;
    
    priv->headset_mic_gain = val;
    dev_dbg(dev, "Headset mic gain set to 0x%02x (%+d dB)\n", 
            (u8)val, mic_gain_hw_to_db((u8)val));
    return count;
}
static DEVICE_ATTR_RW(fpga_headset_mic_gain);

static ssize_t fpga_front_mic_gain_show(struct device *dev,
                                         struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 gain;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_FRONT_MIC_GAIN, &gain);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (%u) = %+d dB\n", 
                   gain, gain, mic_gain_hw_to_db(gain));
}

static ssize_t fpga_front_mic_gain_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_FRONT_MIC_GAIN, (u8)val);
    if (ret < 0)
        return ret;
    
    priv->front_mic_gain = val;
    dev_dbg(dev, "Front mic gain set to 0x%02x (%+d dB)\n", 
            (u8)val, mic_gain_hw_to_db((u8)val));
    return count;
}
static DEVICE_ATTR_RW(fpga_front_mic_gain);

static ssize_t fpga_hp_threshold_show(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 threshold;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_HP_THRESHOLD, &threshold);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (%u)\n", threshold, threshold);
}

static ssize_t fpga_hp_threshold_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_HP_THRESHOLD, (u8)val);
    if (ret < 0)
        return ret;
    
    dev_dbg(dev, "Headphone detection threshold set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(fpga_hp_threshold);

static ssize_t fpga_sai_loop_show(struct device *dev,
                                   struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 sysctrl;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, &sysctrl);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "%d\n", 
                     !!(sysctrl & FPGA_SYSCTRL_SAI_LOOP));
}

static ssize_t fpga_sai_loop_store(struct device *dev,
                                    struct device_attribute *attr,
                                    const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    u8 sysctrl;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 1)
        return -EINVAL;
    
    mutex_lock(&priv->hw_lock);  /* Protect against race conditions */
    
    /* Read current value */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, &sysctrl);
    if (ret < 0) {
        mutex_unlock(&priv->hw_lock);
        return ret;
    }
    
    /* Modify only SAI_LOOP bit */
    if (val)
        sysctrl |= FPGA_SYSCTRL_SAI_LOOP;
    else
        sysctrl &= ~FPGA_SYSCTRL_SAI_LOOP;
    
    /* Write back */
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, sysctrl);
    
    mutex_unlock(&priv->hw_lock);
    
    if (ret < 0)
        return ret;
    
    dev_dbg(dev, "SAI_LOOP %s\n", val ? "enabled" : "disabled");
    return count;
}
static DEVICE_ATTR_RW(fpga_sai_loop);

/* ============================================================================
 * PCM1748 Debug sysfs Interface
 * ============================================================================ */

static ssize_t pcm1748_left_volume_show(struct device *dev,
                                         struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (%u)\n", priv->hp_left_vol, priv->hp_left_vol);
}

static ssize_t pcm1748_left_volume_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = pcm1748_write_reg(priv->pcm1748_spi, PCM1748_LEFT_VOLUME, (u8)val);
    if (ret < 0)
        return ret;
    
    priv->hp_left_vol = val;
    dev_dbg(dev, "PCM1748 Left Volume set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(pcm1748_left_volume);

static ssize_t pcm1748_right_volume_show(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (%u)\n", priv->hp_right_vol, priv->hp_right_vol);
}

static ssize_t pcm1748_right_volume_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = pcm1748_write_reg(priv->pcm1748_spi, PCM1748_RIGHT_VOLUME, (u8)val);
    if (ret < 0)
        return ret;
    
    priv->hp_right_vol = val;
    dev_dbg(dev, "PCM1748 Right Volume set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(pcm1748_right_volume);

static ssize_t pcm1748_format_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    /* PCM1748 has no read capability - show what was last written */
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 val = priv->pcm1748_format;
    const char *fmt_str;
    
    switch (val & 0x07) {
        case 0: fmt_str = "16-bit Right Justified"; break;
        case 5: fmt_str = "24-bit Left Justified"; break;
        case 6: fmt_str = "24-bit I2S"; break;
        default: fmt_str = "Unknown"; break;
    }
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (%s)\n"
                   "Note: PCM1748 is write-only, showing cached value\n",
                   val, fmt_str);
}

static ssize_t pcm1748_format_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = pcm1748_write_reg(priv->pcm1748_spi, PCM1748_FORMAT_CTRL, (u8)val);
    if (ret < 0)
        return ret;
    
    priv->pcm1748_format = val;  /* Cache the written value */
    dev_dbg(dev, "PCM1748 Format Control set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(pcm1748_format);

static ssize_t pcm1748_mute_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    /* PCM1748 has no read capability - show what was last written */
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (L:%s R:%s)\n"
                   "Note: PCM1748 is write-only, showing cached value\n",
                   priv->pcm1748_mute,
                   (priv->pcm1748_mute & 0x01) ? "Muted" : "Unmuted",
                   (priv->pcm1748_mute & 0x02) ? "Muted" : "Unmuted");
}

static ssize_t pcm1748_mute_store(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = pcm1748_write_reg(priv->pcm1748_spi, PCM1748_SOFT_MUTE, (u8)val);
    if (ret < 0)
        return ret;
    
    priv->pcm1748_mute = val;  /* Cache the written value */
    dev_dbg(dev, "PCM1748 Soft Mute set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(pcm1748_mute);

/* Add DAC Control attribute */
static ssize_t pcm1748_dac_ctrl_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 val = priv->pcm1748_dac_ctrl;
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (De-emph:%s, Rate:%dkHz, L_DAC:%s, R_DAC:%s)\n"
                   "Note: PCM1748 is write-only, showing cached value\n",
                   val,
                   (val & 0x10) ? "ON" : "OFF",
                   ((val >> 5) & 0x03) == 1 ? 48 : 
                   ((val >> 5) & 0x03) == 2 ? 44 : 32,
                   (val & 0x01) ? "Disabled" : "Enabled",
                   (val & 0x02) ? "Disabled" : "Enabled");
}

static ssize_t pcm1748_dac_ctrl_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = pcm1748_write_reg(priv->pcm1748_spi, PCM1748_DAC_CTRL, (u8)val);
    if (ret < 0)
        return ret;
    
    priv->pcm1748_dac_ctrl = val;  /* Cache the written value */
    dev_dbg(dev, "PCM1748 DAC Control (0x13) set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(pcm1748_dac_ctrl);

/* Update dump function with correct info */
static ssize_t pcm1748_dump_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    int len = 0;
    int left_db_tenths, right_db_tenths;
    
    /* Calculate attenuation in tenths of dB (to show one decimal place)
     * Formula: Attenuation = 0.5 * (Val - 255) dB
     * We multiply by 10 to get tenths: (Val - 255) * 5 */
    left_db_tenths = ((int)priv->hp_left_vol - 255) * 5;
    right_db_tenths = ((int)priv->hp_right_vol - 255) * 5;
    
    len += snprintf(buf + len, PAGE_SIZE - len, "=== PCM1748 Register Dump ===\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "Note: PCM1748 is write-only, showing initialized values\n\n");
    
    len += snprintf(buf + len, PAGE_SIZE - len, "Reg 16 (0x10) LEFT_VOLUME:   0x%02x (%3u) = %d.%d dB\n", 
                   priv->hp_left_vol, priv->hp_left_vol, 
                   left_db_tenths / 10, abs(left_db_tenths % 10));
    
    len += snprintf(buf + len, PAGE_SIZE - len, "Reg 17 (0x11) RIGHT_VOLUME:  0x%02x (%3u) = %d.%d dB\n", 
                   priv->hp_right_vol, priv->hp_right_vol,
                   right_db_tenths / 10, abs(right_db_tenths % 10));
    
    len += snprintf(buf + len, PAGE_SIZE - len, "Reg 18 (0x12) SOFT_MUTE:     0x00 (Both unmuted, 64x oversampling)\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "Reg 19 (0x13) DAC_CTRL:      0x30 (DACs enabled, De-emph ON, 48kHz)\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "Reg 20 (0x14) FORMAT_CTRL:   0x05 (24-bit Left Justified)\n");
    
    len += snprintf(buf + len, PAGE_SIZE - len, "\n--- Register Details ---\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "  Volume: 0x00=mute to 0xFF=0dB (0.5dB steps)\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "  Mute   (0x12): bit0=L, bit1=R (1=muted)\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "  DAC    (0x13): bit0=L_DAC, bit1=R_DAC (1=disabled)\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "                 bit4=De-emph, bits[6:5]=Rate (01=48kHz)\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "  Format (0x14): 000=16bit RJ, 101=24bit LJ, 110=24bit I2S\n");
 
    if (len >= PAGE_SIZE)
        len = PAGE_SIZE - 1;
    
    return len;
}
static DEVICE_ATTR_RO(pcm1748_dump);

/* ============================================================================
 * TAS5733L Debug SYSFS Interface
 * ============================================================================ */

static ssize_t tas5733_device_id_show(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 device_id;
    int ret;
    
    if (!priv->tas5733_client)
        return scnprintf(buf, PAGE_SIZE, "Error: TAS5733 not available\n");
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_DEVICE_ID, &device_id);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x\n", device_id);
}
static DEVICE_ATTR_RO(tas5733_device_id);

static ssize_t tas5733_error_status_show(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 error_status;
    int ret;
    
    if (!priv->tas5733_client)
        return scnprintf(buf, PAGE_SIZE, "Error: TAS5733 not available\n");
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_ERROR_STATUS, &error_status);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (OCE:%d DCE:%d OTE:%d)\n",
                   error_status,
                   !!(error_status & BIT(4)),  /* Over-current error */
                   !!(error_status & BIT(3)),  /* DC error */
                   !!(error_status & BIT(2))); /* Over-temperature error */
}

static ssize_t tas5733_error_status_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    if (!priv->tas5733_client)
        return -ENODEV;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = tas5733_write_reg(priv->tas5733_client, TAS5733_ERROR_STATUS, (u8)val);
    if (ret < 0)
        return ret;
    
    dev_dbg(dev, "TAS5733 error status cleared with 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(tas5733_error_status);

static ssize_t tas5733_left_volume_show(struct device *dev,
                                         struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u16 volume;
    int ret;
    
    if (!priv->tas5733_client)
        return scnprintf(buf, PAGE_SIZE, "Error: TAS5733 not available\n");
    
    ret = tas5733_read_reg16(priv->tas5733_client, TAS5733_LEFT_VOL, &volume);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%04x (%u)\n", volume, volume);
}

static ssize_t tas5733_left_volume_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    if (!priv->tas5733_client)
        return -ENODEV;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFFFF)
        return -EINVAL;
    
    ret = tas5733_write_reg16(priv->tas5733_client, TAS5733_LEFT_VOL, (u16)val);
    if (ret < 0)
        return ret;
    
    priv->speaker_left_vol = val;
    dev_dbg(dev, "TAS5733 left volume set to 0x%04x\n", (u16)val);
    return count;
}
static DEVICE_ATTR_RW(tas5733_left_volume);

static ssize_t tas5733_right_volume_show(struct device *dev,
                                          struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u16 volume;
    int ret;
    
    if (!priv->tas5733_client)
        return scnprintf(buf, PAGE_SIZE, "Error: TAS5733 not available\n");
    
    ret = tas5733_read_reg16(priv->tas5733_client, TAS5733_RIGHT_VOL, &volume);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%04x (%u)\n", volume, volume);
}

static ssize_t tas5733_right_volume_store(struct device *dev,
                                           struct device_attribute *attr,
                                           const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    if (!priv->tas5733_client)
        return -ENODEV;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFFFF)
        return -EINVAL;
    
    ret = tas5733_write_reg16(priv->tas5733_client, TAS5733_RIGHT_VOL, (u16)val);
    if (ret < 0)
        return ret;
    
    priv->speaker_right_vol = val;
    dev_dbg(dev, "TAS5733 right volume set to 0x%04x\n", (u16)val);
    return count;
}
static DEVICE_ATTR_RW(tas5733_right_volume);

static ssize_t tas5733_master_volume_show(struct device *dev,
                                           struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u16 volume;
    int ret;
    
    if (!priv->tas5733_client)
        return scnprintf(buf, PAGE_SIZE, "Error: TAS5733 not available\n");
    
    ret = tas5733_read_reg16(priv->tas5733_client, TAS5733_MASTER_VOL, &volume);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%04x (%u)\n", volume, volume);
}

static ssize_t tas5733_master_volume_store(struct device *dev,
                                            struct device_attribute *attr,
                                            const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    if (!priv->tas5733_client)
        return -ENODEV;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFFFF)
        return -EINVAL;
    
    ret = tas5733_write_reg16(priv->tas5733_client, TAS5733_MASTER_VOL, (u16)val);
    if (ret < 0)
        return ret;
    
    dev_dbg(dev, "TAS5733 master volume set to 0x%04x\n", (u16)val);
    return count;
}
static DEVICE_ATTR_RW(tas5733_master_volume);

static ssize_t tas5733_soft_mute_show(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 mute;
    int ret;
    
    if (!priv->tas5733_client)
        return scnprintf(buf, PAGE_SIZE, "Error: TAS5733 not available\n");
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_SOFT_MUTE, &mute);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (R:%s L:%s)\n",
                   mute,
                   (mute & BIT(1)) ? "Muted" : "Unmuted",
                   (mute & BIT(0)) ? "Muted" : "Unmuted");
}

static ssize_t tas5733_soft_mute_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    if (!priv->tas5733_client)
        return -ENODEV;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = tas5733_write_reg(priv->tas5733_client, TAS5733_SOFT_MUTE, (u8)val);
    if (ret < 0)
        return ret;
    
    dev_dbg(dev, "TAS5733 soft mute set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(tas5733_soft_mute);

static ssize_t tas5733_sys_ctrl2_show(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 ctrl;
    int ret;
    
    if (!priv->tas5733_client)
        return scnprintf(buf, PAGE_SIZE, "Error: TAS5733 not available\n");
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_SYS_CTRL2, &ctrl);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (HiZ:%s)\n",
                   ctrl,
                   (ctrl & BIT(4)) ? "Enabled" : "Disabled");
}

static ssize_t tas5733_sys_ctrl2_store(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    int ret;
    
    if (!priv->tas5733_client)
        return -ENODEV;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0xFF)
        return -EINVAL;
    
    ret = tas5733_write_reg(priv->tas5733_client, TAS5733_SYS_CTRL2, (u8)val);
    if (ret < 0)
        return ret;
    
    dev_dbg(dev, "TAS5733 SYS_CTRL2 set to 0x%02x\n", (u8)val);
    return count;
}
static DEVICE_ATTR_RW(tas5733_sys_ctrl2);

static ssize_t tas5733_dump_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    int len = 0;
    u8 val8;
    u16 val16;
    int ret;
    
    if (!priv->tas5733_client) {
        return scnprintf(buf, PAGE_SIZE, "Error: TAS5733 not available\n");
    }
    
    len += snprintf(buf + len, PAGE_SIZE - len, "=== TAS5733L Register Dump ===\n\n");
    
    /* 8-bit registers */
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_CLOCK_CTRL, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x00 CLOCK_CTRL:    0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_DEVICE_ID, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x01 DEVICE_ID:     0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_ERROR_STATUS, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x02 ERROR_STATUS:  0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_SYS_CTRL1, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x03 SYS_CTRL1:     0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_AUDIO_FORMAT, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x04 AUDIO_FORMAT:  0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_SYS_CTRL2, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x05 SYS_CTRL2:     0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_SOFT_MUTE, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x06 SOFT_MUTE:     0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    /* 16-bit volume registers */
    ret = tas5733_read_reg16(priv->tas5733_client, TAS5733_MASTER_VOL, &val16);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x07 MASTER_VOL:    0x%04x (%u)\n", 
                   ret < 0 ? 0xFFFF : val16, ret < 0 ? 0xFFFF : val16);
    
    ret = tas5733_read_reg16(priv->tas5733_client, TAS5733_LEFT_VOL, &val16);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x08 LEFT_VOL:      0x%04x (%u)\n", 
                   ret < 0 ? 0xFFFF : val16, ret < 0 ? 0xFFFF : val16);
    
    ret = tas5733_read_reg16(priv->tas5733_client, TAS5733_RIGHT_VOL, &val16);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x09 RIGHT_VOL:     0x%04x (%u)\n", 
                   ret < 0 ? 0xFFFF : val16, ret < 0 ? 0xFFFF : val16);
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_TRIM_REG, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x1B TRIM_REG:      0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    ret = tas5733_read_reg(priv->tas5733_client, TAS5733_MOD_SCHEME, &val8);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x20 MOD_SCHEME:    0x%02x\n", 
                   ret < 0 ? 0xFF : val8);
    
    if (len >= PAGE_SIZE)
        len = PAGE_SIZE - 1;
    
    return len;
}
static DEVICE_ATTR_RO(tas5733_dump);


static ssize_t irq_count_show(struct device *dev,
                                struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    
    return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&priv->irq_count));
}
static DEVICE_ATTR_RO(irq_count);

static ssize_t gpio_irq_value_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    int value;
    
    if (!priv->fpga_irq_gpio)
        return scnprintf(buf, PAGE_SIZE, "Not available\n");
    
    value = gpiod_get_value(priv->fpga_irq_gpio);
    return scnprintf(buf, PAGE_SIZE, "%d\n", value);
}
static DEVICE_ATTR_RO(gpio_irq_value);

static ssize_t headphones_connected_show(struct device *dev,
                                           struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    
    return scnprintf(buf, PAGE_SIZE, "%d\n", priv->headphones_connected);
}
static DEVICE_ATTR_RO(headphones_connected);

static ssize_t dump_all_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 status, int_status, int_mask, int_enable, sysctrl, fw_ver;
    u8 pcb_ver, audio_ctrl, audio_status;
    int gpio_val, len = 0;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_STATUS, &status);
    if (ret < 0) status = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, &int_status);
    if (ret < 0) int_status = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_MASK, &int_mask);
    if (ret < 0) int_mask = 0xFF;
    int_enable = ~int_mask;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, &sysctrl);
    if (ret < 0) sysctrl = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_FW_VERSION, &fw_ver);
    if (ret < 0) fw_ver = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_PCB_VERSION, &pcb_ver);
    if (ret < 0) pcb_ver = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_AUDIO_CTRL0, &audio_ctrl);
    if (ret < 0) audio_ctrl = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_AUDIO_STATUS0, &audio_status);
    if (ret < 0) audio_status = 0xFF;
    
    gpio_val = priv->fpga_irq_gpio ? gpiod_get_value(priv->fpga_irq_gpio) : -1;
    
    len += snprintf(buf + len, PAGE_SIZE - len, "FPGA Register Dump\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "==================\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "FW Version:       0x%02x (%u)\n", fw_ver, fw_ver);
    len += snprintf(buf + len, PAGE_SIZE - len, "PCB Version:      0x%02x (%u)\n", pcb_ver, pcb_ver);
    len += snprintf(buf + len, PAGE_SIZE - len, "STATUS (0x08):    0x%02x  HP:%d BTN1:%d BTN0:%d\n",
                   status,
                   !!(status & FPGA_STATUS_HP_DETECT),
                   !!(status & FPGA_STATUS_BUTTON1),
                   !!(status & FPGA_STATUS_BUTTON0));
    len += snprintf(buf + len, PAGE_SIZE - len, "INT_STATUS (0x09):0x%02x  HP:%d ENC1:%d BTN1:%d ENC0:%d BTN0:%d\n",
                   int_status,
                   !!(int_status & FPGA_INT_HP_DETECT),
                   !!(int_status & FPGA_INT_ENCODER1),
                   !!(int_status & FPGA_INT_BUTTON1),
                   !!(int_status & FPGA_INT_ENCODER0),
                   !!(int_status & FPGA_INT_BUTTON0));
    len += snprintf(buf + len, PAGE_SIZE - len, "INT_ENABLE (0x0E):0x%02x  HP:%d ENC1:%d BTN1:%d ENC0:%d BTN0:%d\n",
                   int_enable,
                   !!(int_enable & FPGA_INT_MSK_HP_DETECT),
                   !!(int_enable & FPGA_INT_MSK_ENCODER1),
                   !!(int_enable & FPGA_INT_MSK_BUTTON1),
                   !!(int_enable & FPGA_INT_MSK_ENCODER0),
                   !!(int_enable & FPGA_INT_MSK_BUTTON0));
    len += snprintf(buf + len, PAGE_SIZE - len, "SYSCTRL (0x0A):   0x%02x  EMB_MIC:%d HP_MIC:%d HP:%d SPK:%d SAI_LOOP:%d\n",
                   sysctrl,
                   !!(sysctrl & FPGA_SYSCTRL_FRONT_MIC_EN),
                   !!(sysctrl & FPGA_SYSCTRL_HEADSET_MIC_EN),
                   !!(sysctrl & FPGA_SYSCTRL_HEADSET_EN),
                   !!(sysctrl & FPGA_SYSCTRL_SPEAKER_EN),
                   !!(sysctrl & FPGA_SYSCTRL_SAI_LOOP));
    len += snprintf(buf + len, PAGE_SIZE - len, "AUDIO_CTRL (0x11):0x%02x  12V_PD:%d FMT:%d MD1:%d MD0:%d\n",
                   audio_ctrl,
                   !!(audio_ctrl & FPGA_AUDIO_12V_PD),
                   !!(audio_ctrl & FPGA_AUDIO_SPKR_FMT),
                   !!(audio_ctrl & FPGA_AUDIO_SPKR_MD1),
                   !!(audio_ctrl & FPGA_AUDIO_SPKR_MD0));
    len += snprintf(buf + len, PAGE_SIZE - len, "AUDIO_STAT (0x12):0x%02x  ZEROR:%d ZEROL:%d\n",
                   audio_status,
                   !!(audio_status & FPGA_AUDIO_DAC_ZEROR),
                   !!(audio_status & FPGA_AUDIO_DAC_ZEROL));
    len += snprintf(buf + len, PAGE_SIZE - len, "\nDriver State\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "============\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "IRQ Count:        %d\n", atomic_read(&priv->irq_count));
    len += snprintf(buf + len, PAGE_SIZE - len, "GPIO IRQ Value:   %d\n", gpio_val);
    len += snprintf(buf + len, PAGE_SIZE - len, "HP Connected:     %d\n", priv->headphones_connected);
    len += snprintf(buf + len, PAGE_SIZE - len, "Audio Path:       %s\n", priv->audio_path_local ? "Speaker" : "Headset");
    
    if (len >= PAGE_SIZE)
        len = PAGE_SIZE - 1;
    
    return len;
}
static DEVICE_ATTR_RO(dump_all);

static ssize_t poll_status_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 status;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_STATUS, &status);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x  HP:%d BTN1:%d BTN0:%d AUX_PWR:%d POE_PWR:%d\n",
                   status,
                   !!(status & FPGA_STATUS_HP_DETECT),
                   !!(status & FPGA_STATUS_BUTTON1),
                   !!(status & FPGA_STATUS_BUTTON0),
                   !!(status & FPGA_STATUS_AUX_PWR),
                   !!(status & FPGA_STATUS_POE_PWR));
}
static DEVICE_ATTR_RO(poll_status);

static ssize_t poll_encoders_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 encoder0, encoder1;
    int ret;
    int len = 0;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_ENCODER0, &encoder0);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error reading encoder0: %d\n", ret);
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_ENCODER1, &encoder1);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error reading encoder1: %d\n", ret);
    
    len += snprintf(buf + len, PAGE_SIZE - len, "Encoder0 (Right): 0x%02x (%d)\n", 
                   encoder0, (int8_t)encoder0);
    len += snprintf(buf + len, PAGE_SIZE - len, "Encoder1 (Left):  0x%02x (%d)\n", 
                   encoder1, (int8_t)encoder1);

    if (len >= PAGE_SIZE)
        len = PAGE_SIZE - 1;
         
    return len;
}
static DEVICE_ATTR_RO(poll_encoders);

static ssize_t poll_all_show(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 led1_r, led1_g, led1_b, led2_r, led2_g, led2_b;
    u8 encoder0, encoder1, status, int_status, sysctrl;
    u8 headset_mic_gain, front_mic_gain, hp_threshold, int_mask, int_enable, fw_ver;
    u8 pcb_ver, audio_ctrl, audio_status;
    int ret;
    int len = 0;
    
    /* Read all FPGA registers */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_LED1_R, &led1_r);
    if (ret < 0) led1_r = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_LED1_G, &led1_g);
    if (ret < 0) led1_g = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_LED1_B, &led1_b);
    if (ret < 0) led1_b = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_LED2_R, &led2_r);
    if (ret < 0) led2_r = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_LED2_G, &led2_g);
    if (ret < 0) led2_g = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_LED2_B, &led2_b);
    if (ret < 0) led2_b = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_ENCODER0, &encoder0);
    if (ret < 0) encoder0 = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_ENCODER1, &encoder1);
    if (ret < 0) encoder1 = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_STATUS, &status);
    if (ret < 0) status = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_STATUS, &int_status);
    if (ret < 0) int_status = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_SYSCTRL, &sysctrl);
    if (ret < 0) sysctrl = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_HEADSET_MIC_GAIN, &headset_mic_gain);
    if (ret < 0) headset_mic_gain = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_FRONT_MIC_GAIN, &front_mic_gain);
    if (ret < 0) front_mic_gain = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_HP_THRESHOLD, &hp_threshold);
    if (ret < 0) hp_threshold = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_INT_MASK, &int_mask);
    if (ret < 0) int_mask = 0xFF;
    int_enable = ~int_mask;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_FW_VERSION, &fw_ver);
    if (ret < 0) fw_ver = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_PCB_VERSION, &pcb_ver);
    if (ret < 0) pcb_ver = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_AUDIO_CTRL0, &audio_ctrl);
    if (ret < 0) audio_ctrl = 0xFF;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_AUDIO_STATUS0, &audio_status);
    if (ret < 0) audio_status = 0xFF;
    
    /* Format output */
    len += snprintf(buf + len, PAGE_SIZE - len, "=== FPGA Complete Register Dump ===\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* Version info */
    len += snprintf(buf + len, PAGE_SIZE - len, "0x0F FW_VERSION:     0x%02x (%u)\n", fw_ver, fw_ver);
    len += snprintf(buf + len, PAGE_SIZE - len, "0x10 PCB_VERSION:    0x%02x (%u)\n", pcb_ver, pcb_ver);
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* LED registers */
    len += snprintf(buf + len, PAGE_SIZE - len, "LED Registers:\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x00 LED1_R:       0x%02x (%3u)\n", led1_r, led1_r);
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x01 LED1_G:       0x%02x (%3u)\n", led1_g, led1_g);
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x02 LED1_B:       0x%02x (%3u)\n", led1_b, led1_b);
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x03 LED2_R:       0x%02x (%3u)\n", led2_r, led2_r);
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x04 LED2_G:       0x%02x (%3u)\n", led2_g, led2_g);
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x05 LED2_B:       0x%02x (%3u)\n", led2_b, led2_b);
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* Encoder registers */
    len += snprintf(buf + len, PAGE_SIZE - len, "Encoder Registers (delta values):\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x06 ENCODER0:     0x%02x (%4d) [Right rotary]\n", 
                   encoder0, (int8_t)encoder0);
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x07 ENCODER1:     0x%02x (%4d) [Left rotary]\n", 
                   encoder1, (int8_t)encoder1);
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* Status register with bit breakdown */
    len += snprintf(buf + len, PAGE_SIZE - len, "0x08 STATUS:        0x%02x (bin: %c%c%c%c%c%c%c%c)\n",
                   status,
                   (status & BIT(7)) ? '1' : '0',
                   (status & BIT(6)) ? '1' : '0',
                   (status & BIT(5)) ? '1' : '0',
                   (status & BIT(4)) ? '1' : '0',
                   (status & BIT(3)) ? '1' : '0',
                   (status & BIT(2)) ? '1' : '0',
                   (status & BIT(1)) ? '1' : '0',
                   (status & BIT(0)) ? '1' : '0');
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit6 AUX_PWR:    %d\n", !!(status & FPGA_STATUS_AUX_PWR));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit5 POE_PWR:    %d\n", !!(status & FPGA_STATUS_POE_PWR));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit4 HP_DETECT:  %d %s\n", 
                   !!(status & FPGA_STATUS_HP_DETECT),
                   (status & FPGA_STATUS_HP_DETECT) ? "(INSERTED)" : "(removed)");
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit2 BUTTON1:    %d %s\n", 
                   !!(status & FPGA_STATUS_BUTTON1),
                   (status & FPGA_STATUS_BUTTON1) ? "(not pressed)" : "(PRESSED)");
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit0 BUTTON0:    %d %s\n", 
                   !!(status & FPGA_STATUS_BUTTON0),
                   (status & FPGA_STATUS_BUTTON0) ? "(not pressed)" : "(PRESSED)");
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* Interrupt status register */
    len += snprintf(buf + len, PAGE_SIZE - len, "0x09 INT_STATUS:    0x%02x (bin: %c%c%c%c%c%c%c%c)\n",
                   int_status,
                   (int_status & BIT(7)) ? '1' : '0',
                   (int_status & BIT(6)) ? '1' : '0',
                   (int_status & BIT(5)) ? '1' : '0',
                   (int_status & BIT(4)) ? '1' : '0',
                   (int_status & BIT(3)) ? '1' : '0',
                   (int_status & BIT(2)) ? '1' : '0',
                   (int_status & BIT(1)) ? '1' : '0',
                   (int_status & BIT(0)) ? '1' : '0');
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit4 HP_INT:     %d\n", !!(int_status & FPGA_INT_HP_DETECT));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit3 ENC1_INT:   %d\n", !!(int_status & FPGA_INT_ENCODER1));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit2 BTN1_INT:   %d\n", !!(int_status & FPGA_INT_BUTTON1));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit1 ENC0_INT:   %d\n", !!(int_status & FPGA_INT_ENCODER0));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit0 BTN0_INT:   %d\n", !!(int_status & FPGA_INT_BUTTON0));
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* System control register */
    len += snprintf(buf + len, PAGE_SIZE - len, "0x0A SYSCTRL:       0x%02x (bin: %c%c%c%c%c%c%c%c)\n",
                   sysctrl,
                   (sysctrl & BIT(7)) ? '1' : '0',
                   (sysctrl & BIT(6)) ? '1' : '0',
                   (sysctrl & BIT(5)) ? '1' : '0',
                   (sysctrl & BIT(4)) ? '1' : '0',
                   (sysctrl & BIT(3)) ? '1' : '0',
                   (sysctrl & BIT(2)) ? '1' : '0',
                   (sysctrl & BIT(1)) ? '1' : '0',
                   (sysctrl & BIT(0)) ? '1' : '0');
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit7 HP_MIC:     %d\n", !!(sysctrl & FPGA_SYSCTRL_HEADSET_MIC_EN));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit6 FRONT_MIC:  %d\n", !!(sysctrl & FPGA_SYSCTRL_FRONT_MIC_EN));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit5 HEADSET:    %d\n", !!(sysctrl & FPGA_SYSCTRL_HEADSET_EN));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit4 SPEAKER:    %d\n", !!(sysctrl & FPGA_SYSCTRL_SPEAKER_EN));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit3 SAI_LOOP:   %d\n", !!(sysctrl & FPGA_SYSCTRL_SAI_LOOP));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit2 ENC1_RST:   %d\n", !!(sysctrl & FPGA_SYSCTRL_ENC1_RST));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit1 ENC0_RST:   %d\n", !!(sysctrl & FPGA_SYSCTRL_ENC0_RST));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit0 SYS_RST:    %d\n", !!(sysctrl & FPGA_SYSCTRL_SYS_RST));
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* Audio configuration registers */
    len += snprintf(buf + len, PAGE_SIZE - len, "Audio Config:\n");
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x0B HEADSET_MIC_GAIN:  hw=0x%02x (%+ddB)\n", 
                   headset_mic_gain, mic_gain_hw_to_db(headset_mic_gain));
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x0C FRONT_MIC_GAIN:    hw=0x%02x (%+ddB)\n", 
                   front_mic_gain, mic_gain_hw_to_db(front_mic_gain));
    len += snprintf(buf + len, PAGE_SIZE - len, "  0x0D HP_THRESHOLD:      0x%02x (%3u)\n", 
                   hp_threshold, hp_threshold);
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* Interrupt enable register */
    len += snprintf(buf + len, PAGE_SIZE - len, "0x0E INT_ENABLE:    0x%02x (bin: %c%c%c%c%c%c%c%c)\n",
                   int_enable,
                   (int_enable & BIT(7)) ? '1' : '0',
                   (int_enable & BIT(6)) ? '1' : '0',
                   (int_enable & BIT(5)) ? '1' : '0',
                   (int_enable & BIT(4)) ? '1' : '0',
                   (int_enable & BIT(3)) ? '1' : '0',
                   (int_enable & BIT(2)) ? '1' : '0',
                   (int_enable & BIT(1)) ? '1' : '0',
                   (int_enable & BIT(0)) ? '1' : '0');
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit4 HP_EN:      %d\n", !!(int_enable & FPGA_INT_MSK_HP_DETECT));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit3 ENC1_EN:    %d\n", !!(int_enable & FPGA_INT_MSK_ENCODER1));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit2 BTN1_EN:    %d\n", !!(int_enable & FPGA_INT_MSK_BUTTON1));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit1 ENC0_EN:    %d\n", !!(int_enable & FPGA_INT_MSK_ENCODER0));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit0 BTN0_EN:    %d\n", !!(int_enable & FPGA_INT_MSK_BUTTON0));
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* Audio control register */
    len += snprintf(buf + len, PAGE_SIZE - len, "0x11 AUDIO_CTRL0:   0x%02x (bin: %c%c%c%c%c%c%c%c)\n",
                   audio_ctrl,
                   (audio_ctrl & BIT(7)) ? '1' : '0',
                   (audio_ctrl & BIT(6)) ? '1' : '0',
                   (audio_ctrl & BIT(5)) ? '1' : '0',
                   (audio_ctrl & BIT(4)) ? '1' : '0',
                   (audio_ctrl & BIT(3)) ? '1' : '0',
                   (audio_ctrl & BIT(2)) ? '1' : '0',
                   (audio_ctrl & BIT(1)) ? '1' : '0',
                   (audio_ctrl & BIT(0)) ? '1' : '0');
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit7 12V_PD:     %d %s\n", 
                   !!(audio_ctrl & FPGA_AUDIO_12V_PD),
                   (audio_ctrl & FPGA_AUDIO_12V_PD) ? "(powered down)" : "(ENABLED)");
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit2 SPKR_FMT:   %d\n", !!(audio_ctrl & FPGA_AUDIO_SPKR_FMT));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit1 SPKR_MD1:   %d\n", !!(audio_ctrl & FPGA_AUDIO_SPKR_MD1));
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit0 SPKR_MD0:   %d\n", !!(audio_ctrl & FPGA_AUDIO_SPKR_MD0));
    len += snprintf(buf + len, PAGE_SIZE - len, "\n");
    
    /* Audio status register */
    len += snprintf(buf + len, PAGE_SIZE - len, "0x12 AUDIO_STATUS0: 0x%02x (bin: %c%c%c%c%c%c%c%c)\n",
                   audio_status,
                   (audio_status & BIT(7)) ? '1' : '0',
                   (audio_status & BIT(6)) ? '1' : '0',
                   (audio_status & BIT(5)) ? '1' : '0',
                   (audio_status & BIT(4)) ? '1' : '0',
                   (audio_status & BIT(3)) ? '1' : '0',
                   (audio_status & BIT(2)) ? '1' : '0',
                   (audio_status & BIT(1)) ? '1' : '0',
                   (audio_status & BIT(0)) ? '1' : '0');
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit1 DAC_ZEROR:  %d %s\n",
                   !!(audio_status & FPGA_AUDIO_DAC_ZEROR),
                   (audio_status & FPGA_AUDIO_DAC_ZEROR) ? "(ZERO)" : "(active)");
    len += snprintf(buf + len, PAGE_SIZE - len, "  bit0 DAC_ZEROL:  %d %s\n",
                   !!(audio_status & FPGA_AUDIO_DAC_ZEROL),
                   (audio_status & FPGA_AUDIO_DAC_ZEROL) ? "(ZERO)" : "(active)");

    if (len >= PAGE_SIZE)
        len = PAGE_SIZE - 1;
        
    return len;
}
static DEVICE_ATTR_RO(poll_all);

static ssize_t rotary_position_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    
    return scnprintf(buf, PAGE_SIZE, "Left: %3d; Right: %3d\n", 
                   priv->left_rotary_position,
                   priv->right_rotary_position);
}
static DEVICE_ATTR_RO(rotary_position);

/* PCB Version */
static ssize_t fpga_pcb_version_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 version;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_PCB_VERSION, &version);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "%u (0x%02x)\n", version, version);
}
static DEVICE_ATTR_RO(fpga_pcb_version);

/* PCM1808 Format Control */
static ssize_t pcm1808_format_show(struct device *dev,
                                         struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 ctrl;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_AUDIO_CTRL0, &ctrl);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (FMT=%d MD1=%d MD0=%d)\n",
                   ctrl & 0x07,
                   !!(ctrl & FPGA_AUDIO_SPKR_FMT),
                   !!(ctrl & FPGA_AUDIO_SPKR_MD1),
                   !!(ctrl & FPGA_AUDIO_SPKR_MD0));
}

static ssize_t pcm1808_format_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    unsigned long val;
    u8 ctrl;
    int ret;
    
    ret = kstrtoul(buf, 0, &val);
    if (ret)
        return ret;
    
    if (val > 0x07)
        return -EINVAL;
    
    /* Read current value, modify only low 3 bits */
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_AUDIO_CTRL0, &ctrl);
    if (ret < 0)
        return ret;
    
    ctrl = (ctrl & 0xF8) | (val & 0x07);
    
    ret = fpga_write_reg(priv->fpga_spi, FPGA_REG_AUDIO_CTRL0, ctrl);
    if (ret < 0)
        return ret;
    
    return count;
}
static DEVICE_ATTR_RW(pcm1808_format);

/* PCM1748 Zero Status */
static ssize_t pcm1748_zero_status_show(struct device *dev,
                                              struct device_attribute *attr, char *buf)
{
    struct densitron_audio_priv *priv = dev_get_drvdata(dev);
    u8 status;
    int ret;
    
    ret = fpga_read_reg(priv->fpga_spi, FPGA_REG_AUDIO_STATUS0, &status);
    if (ret < 0)
        return scnprintf(buf, PAGE_SIZE, "Error: %d\n", ret);
    
    return scnprintf(buf, PAGE_SIZE, "0x%02x (R_ZERO=%d L_ZERO=%d)\n",
                   status & 0x03,
                   !!(status & FPGA_AUDIO_DAC_ZEROR),
                   !!(status & FPGA_AUDIO_DAC_ZEROL));
}
static DEVICE_ATTR_RO(pcm1748_zero_status);


static struct attribute *densitron_audio_attrs[] = {
    &dev_attr_fpga_status.attr,
    &dev_attr_fpga_int_status.attr,
    &dev_attr_fpga_int_enable.attr,
    &dev_attr_fpga_sysctrl.attr,
    &dev_attr_fpga_sai_loop.attr,
    &dev_attr_fpga_fw_version.attr,
    &dev_attr_fpga_pcb_version.attr,
    &dev_attr_fpga_headset_mic_gain.attr,
    &dev_attr_fpga_front_mic_gain.attr,
    &dev_attr_fpga_hp_threshold.attr,
    &dev_attr_irq_count.attr,
    &dev_attr_gpio_irq_value.attr,
    &dev_attr_headphones_connected.attr,
    &dev_attr_dump_all.attr,
    &dev_attr_poll_status.attr,
    &dev_attr_poll_encoders.attr,
    &dev_attr_poll_all.attr,    
    /* AMP PCM1808 attributes */
    &dev_attr_pcm1808_format.attr,
    /* PA TAS5733L attributes */
    &dev_attr_tas5733_device_id.attr,
    &dev_attr_tas5733_error_status.attr,
    &dev_attr_tas5733_left_volume.attr,
    &dev_attr_tas5733_right_volume.attr,
    &dev_attr_tas5733_master_volume.attr,
    &dev_attr_tas5733_soft_mute.attr,
    &dev_attr_tas5733_sys_ctrl2.attr,
    &dev_attr_tas5733_dump.attr,
    /* DAC PCM1748 attributes */
    &dev_attr_pcm1748_left_volume.attr,
    &dev_attr_pcm1748_right_volume.attr,
    &dev_attr_pcm1748_mute.attr,
    &dev_attr_pcm1748_dac_ctrl.attr,      
    &dev_attr_pcm1748_format.attr,
    &dev_attr_pcm1748_dump.attr,
    &dev_attr_pcm1748_zero_status.attr,
    /* rotary position (integrator) */
    &dev_attr_rotary_position.attr,
    NULL,
};

static const struct attribute_group densitron_audio_attr_group = {
    .attrs = densitron_audio_attrs,
};

/* ============================================================================
 * Main Driver Probe and Device Tree Parsing
 * ============================================================================ */

static int densitron_audio_platform_probe(struct platform_device *pdev)
{
    struct densitron_audio_priv *priv;
    struct device_node *np = pdev->dev.of_node;
    struct device_node *i2c_node, *fpga_node, *pcm1748_node;
    struct i2c_client *i2c_client;
    int ret;

    dev_dbg(&pdev->dev, "Densitron audio platform driver probe started\n");

    /* Allocate private data */
    priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    platform_set_drvdata(pdev, priv);

    /* Initialize mutex and atomic counters */
    mutex_init(&priv->hw_lock);
    atomic_set(&priv->irq_count, 0);
    atomic_set(&priv->format_rejects, 0);

    /* Get reference to TAS5733 I2C device from device tree */
    i2c_node = of_parse_phandle(np, "tas5733-i2c", 0);
    if (!i2c_node) {
        dev_err(&pdev->dev, "Missing tas5733-i2c reference in device tree\n");
        return -ENODEV;
    }

    i2c_client = of_find_i2c_device_by_node(i2c_node);
    of_node_put(i2c_node);
    
    if (!i2c_client) {
        dev_err(&pdev->dev, "Failed to find TAS5733 I2C device - deferring probe\n");
        return -EPROBE_DEFER;
    }
    
    priv->tas5733_client = i2c_client;

    /* Get references to SPI devices from device tree */
    fpga_node = of_parse_phandle(np, "fpga-spi", 0);
    if (!fpga_node) {
        dev_err(&pdev->dev, "Missing fpga-spi reference in device tree\n");
        return -ENODEV;
    }

    pcm1748_node = of_parse_phandle(np, "pcm1748-spi", 0);
    if (!pcm1748_node) {
        dev_err(&pdev->dev, "Missing pcm1748-spi reference in device tree\n");
        of_node_put(fpga_node);
        return -ENODEV;
    }

    /* Find SPI devices */
    priv->fpga_spi = of_find_spi_device_by_node(fpga_node);
    priv->pcm1748_spi = of_find_spi_device_by_node(pcm1748_node);
    
    of_node_put(fpga_node);
    of_node_put(pcm1748_node);

    if (!priv->fpga_spi || !priv->pcm1748_spi) {
        dev_err(&pdev->dev, "Failed to find SPI devices - deferring probe\n");
        return -EPROBE_DEFER;
    }

    /* Get GPIO descriptors */
    priv->reset_gpio = devm_gpiod_get_optional(&pdev->dev, "reset", GPIOD_OUT_HIGH);
    if (IS_ERR(priv->reset_gpio)) {
        ret = PTR_ERR(priv->reset_gpio);
        dev_err(&pdev->dev, "Failed to get reset GPIO: %d\n", ret);
        return ret;
    }

    priv->fpga_irq_gpio = devm_gpiod_get(&pdev->dev, "fpga-irq", GPIOD_IN);
    if (IS_ERR(priv->fpga_irq_gpio)) {
        ret = PTR_ERR(priv->fpga_irq_gpio);
        dev_err(&pdev->dev, "Failed to get FPGA IRQ GPIO: %d\n", ret);
        return ret;
    }

    /* Get IRQ number but DON'T request it yet */
    priv->fpga_irq = gpiod_to_irq(priv->fpga_irq_gpio);
    if (priv->fpga_irq < 0) {
        dev_err(&pdev->dev, "Failed to get FPGA IRQ: %d\n", priv->fpga_irq);
        return priv->fpga_irq;
    }

    /* Initialize work queue */
    INIT_WORK(&priv->irq_work, fpga_irq_work);

    /* Setup input devices */
    ret = setup_input_devices(priv);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to setup input devices: %d\n", ret);
        return ret;
    }

    /* Setup LED devices */
    ret = setup_led_devices(priv);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to setup LED devices: %d\n", ret);
        return ret;
    }

    /* Initialize default values */
    priv->audio_path_local = true;
    priv->mclk = DENSITRON_MCLK_FREQ;

    /* Register ALSA SoC component - this will call densitron_audio_component_probe()
     * which initializes the hardware */
    ret = devm_snd_soc_register_component(&pdev->dev,
                                          &densitron_audio_component,
                                          &inf1000_dai, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to register ALSA component: %d\n", ret);
        return ret;
    }
    
    /* ========================================================================
     * NOW request the IRQ - hardware is fully initialized at this point
     * ======================================================================== */
    ret = devm_request_irq(&pdev->dev, priv->fpga_irq, fpga_irq_handler,
                           IRQF_TRIGGER_RISING | IRQF_ONESHOT,
                           "densitron-fpga", priv);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to request FPGA IRQ: %d\n", ret);
        return ret;
    }

    dev_dbg(&pdev->dev, "IRQ %d enabled after hardware initialization\n", priv->fpga_irq);

    /* Create sysfs debug interface */
    ret = sysfs_create_group(&pdev->dev.kobj, &densitron_audio_attr_group);
    if (ret < 0) {
        dev_warn(&pdev->dev, "Failed to create sysfs attributes: %d\n", ret);
        /* Non-fatal, continue */
    }
    
    dev_info(&pdev->dev, "Densitron audio driver initialized successfully\n");
    dev_dbg(&pdev->dev, "FPGA MCLK: %d Hz, Format: S24_LE Left Justified, Channels: %d\n",  DENSITRON_MCLK_FREQ, DENSITRON_CHANNELS);
    
    /* Initialize rotary switches accumulators */
    priv->left_rotary_position = 0;
    priv->right_rotary_position = 0;

    return 0;
}

static int densitron_audio_platform_remove(struct platform_device *pdev)
{
    struct densitron_audio_priv *priv = platform_get_drvdata(pdev);
    
    /* Stop all audio operations before teardown */
    if (priv->component && priv->component->card) {
        struct snd_card *card = priv->component->card->snd_card;
        dev_info(&pdev->dev, "Disconnecting audio card...\n");
        snd_card_disconnect_sync(card);
    }
    
    /* Mask all FPGA interrupts immediately */
    if (priv->fpga_spi) {
        fpga_write_reg(priv->fpga_spi, FPGA_REG_INT_MASK, 0xFF);
    }

    /* Mark as uninitialized for potential rebind */
    mutex_lock(&priv->hw_lock);
    priv->hardware_initialized = false;
    mutex_unlock(&priv->hw_lock);
    
    /* Remove sysfs interface */
    sysfs_remove_group(&pdev->dev.kobj, &densitron_audio_attr_group);
    
    /* Cancel work */
    cancel_work_sync(&priv->irq_work);
    
    /* Unregister input devices before freeing priv */
    if (priv->left_rotary) {
        input_unregister_device(priv->left_rotary);
        priv->left_rotary = NULL;
    }
    if (priv->right_rotary) {
        input_unregister_device(priv->right_rotary);
        priv->right_rotary = NULL;
    }
    
    /* Explicitly unregister LED devices
     * Note: LEDs were registered to fpga_spi device, not platform device,
     * so devm_ won't auto-cleanup on platform unbind */
    devm_led_classdev_unregister(&priv->fpga_spi->dev, &priv->led1_r);
    devm_led_classdev_unregister(&priv->fpga_spi->dev, &priv->led1_g);
    devm_led_classdev_unregister(&priv->fpga_spi->dev, &priv->led1_b);
    devm_led_classdev_unregister(&priv->fpga_spi->dev, &priv->led2_r);
    devm_led_classdev_unregister(&priv->fpga_spi->dev, &priv->led2_g);
    devm_led_classdev_unregister(&priv->fpga_spi->dev, &priv->led2_b);
    
    
    // Soft mute speakers and headphones
    if (priv->hardware_initialized && priv->tas5733_client && priv->pcm1748_spi) {
        tas5733_write_reg(priv->tas5733_client, TAS5733_SOFT_MUTE, 0x07);
        pcm1748_write_reg(priv->pcm1748_spi, PCM1748_SOFT_MUTE, 0x01);
    }
    
    /* Print debug statistics */
    dev_dbg(&pdev->dev, "IRQ count: %d, Format rejects: %d\n",
             atomic_read(&priv->irq_count), atomic_read(&priv->format_rejects));
             
    dev_info(&pdev->dev, "Densitron audio driver removed\n");
    
    return 0;
}

/* ============================================================================
 * Module Definition
 * ============================================================================ */

static const struct of_device_id densitron_audio_of_match[] = {
    { .compatible = "densitron,audio" },
    { }
};
MODULE_DEVICE_TABLE(of, densitron_audio_of_match);

static struct platform_driver densitron_audio_platform_driver = {
    .driver = {
        .name = "densitron-audio",
        .of_match_table = densitron_audio_of_match,
    },
    .probe = densitron_audio_platform_probe,
    .remove = densitron_audio_platform_remove,
};

module_platform_driver(densitron_audio_platform_driver);

MODULE_DESCRIPTION("Densitron INF1000 Audio ASoC Driver");
MODULE_AUTHOR("Gian Luca Bernocchi <gianluca.bernocchi@quixant.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("4.13");
