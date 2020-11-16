// Raspberry Pi MCP3202 ADC streaming interface; see https://iosoft.blog for details
//
// Copyright (c) 2020 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// v0.20 JPB 16/11/20 Tidied up for first Github release

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include "rpi_dma_utils.h"

#define VERSION "0.20"

#define SAMPLE_RATE     100     // Default & max sample rate (samples/sec)
#define MAX_SAMPLE_RATE 50000

// PWM definitions: divisor, and reload value
#define PWM_FREQ        1000000
#define PWM_VALUE       2

// ADC sample size (2 bytes, with 11 data bits)
#define ADC_RAW_LEN     2

// ADC and DAC chip-enables
#define ADC_CE_NUM      0
#define DAC_CE_NUM      1

// Definitions for 2 bytes per ADC sample (11-bit)
#define ADC_REQUEST(c)  {0xc0 | (c)<<5, 0x00}
#define ADC_VOLTAGE(n)  (((n) * 3.3) / 2048.0)
#define ADC_MILLIVOLTS(n) ((int)((((n) * 3300) + 1024) / 2048))
#define ADC_RAW_VAL(d)  (((uint16_t)(d)<<8 | (uint16_t)(d)>>8) & 0x7ff)

// Non-cached memory size
#define MAX_SAMPS       1024
#define SAMP_SIZE       4
#define BUFF_LEN        (MAX_SAMPS * SAMP_SIZE)
#define MAX_BUFFS       2
#define VC_MEM_SIZE     (PAGE_SIZE + (BUFF_LEN * MAX_BUFFS))

// DMA control block macros
#define NUM_CBS         10
#define REG(r, a)       REG_BUS_ADDR(r, a)
#define MEM(m, a)       MEM_BUS_ADDR(m, a)
#define CBS(n)          MEM_BUS_ADDR(mp, &dp->cbs[(n)])

// DMA transfer information for PWM and SPI
#define PWM_TI          (DMA_DEST_DREQ | (DMA_PWM_DREQ << 16) | DMA_WAIT_RESP)
#define SPI_RX_TI       (DMA_SRCE_DREQ | (DMA_SPI_RX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_DEST_INC)
#define SPI_TX_TI       (DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC)

// SPI clock frequency
#define MIN_SPI_FREQ    10000
#define MAX_SPI_FREQ    1000000
#define SPI_FREQ        1000000

// SPI 0 pin definitions
#define SPI0_CE0_PIN    8
#define SPI0_CE1_PIN    7
#define SPI0_MISO_PIN   9
#define SPI0_MOSI_PIN   10
#define SPI0_SCLK_PIN   11

// SPI registers and constants
#define SPI0_BASE       (PHYS_REG_BASE + 0x204000)
#define SPI_CS          0x00
#define SPI_FIFO        0x04
#define SPI_CLK         0x08
#define SPI_DLEN        0x0c
#define SPI_DC          0x14
#define SPI_FIFO_CLR    (3 << 4)
#define SPI_RX_FIFO_CLR (2 << 4)
#define SPI_TX_FIFO_CLR (1 << 4)
#define SPI_TFR_ACT     (1 << 7)
#define SPI_DMA_EN      (1 << 8)
#define SPI_AUTO_CS     (1 << 11)
#define SPI_RXD         (1 << 17)
#define SPI_CE0         0
#define SPI_CE1         1

// SPI register strings
char *spi_regstrs[] = {"CS", "FIFO", "CLK", "DLEN", "LTOH", "DC", ""};

// Microsecond timer
#define USEC_BASE       (PHYS_REG_BASE + 0x3000)
#define USEC_TIME       0x04
uint32_t usec_start;

// Buffer for streaming output, and raw Rx data
#define STREAM_BUFFLEN  10000
char stream_buff[STREAM_BUFFLEN];
uint32_t rx_buff[MAX_SAMPS];

// fcntl constant to get free FIFO length
#define F_GETPIPE_SZ    1032

// Virtual memory pointers to acceess peripherals & memory
extern MEM_MAP gpio_regs, dma_regs, clk_regs, pwm_regs;
MEM_MAP vc_mem, spi_regs, usec_regs;

// File descriptor for FIFO
int fifo_fd;

// Data formats for -f option
#define FMT_USEC        1

// Command-line variables
int in_chans=1, sample_count=0, sample_rate=SAMPLE_RATE;
int data_format, testmode, verbose, lockstep;
uint32_t pwm_range, samp_total, overrun_total, fifo_size;
char *fifo_name;

void terminate(int sig);
void map_devices(void);
void get_uncached_mem(MEM_MAP *mp, int size);
int adc_get_sample(int chan);
float test_pwm_frequency(MEM_MAP *mp);
float test_spi_frequency(MEM_MAP *mp);
void adc_dma_init(MEM_MAP *mp, int ns, int single);
void adc_stream_start(void);
void adc_stream_wait(void);
void adc_stream_stop(void);
int adc_stream_csv(MEM_MAP *mp, char *vals, int maxlen, int nsamp);
void dma_wait(int chan);
void do_streaming(MEM_MAP *mp, char *vals, int maxlen, int nsamp);
int create_fifo(char *fname);
int open_fifo_write(char *fname);
int write_fifo(int fd, void *data, int dlen);
uint32_t fifo_freespace(int fd);
void destroy_fifo(char *fname, int fd);
int init_spi(int hz);
void spi_clear(void);
void spi_cs(int set);
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len);
void spi_disable(void);
void disp_spi(void);

// Main program
int main(int argc, char *argv[])
{
    int args=0, f, val=0;
    float freq;

    printf("RPi ADC streamer v" VERSION "\n");
    while (argc > ++args)               // Process command-line args
    {
        if (argv[args][0] == '-')
        {
            switch (toupper(argv[args][1]))
            {
            case 'F':                   // -F: output format
                if (args>=argc-1 || !isdigit((int)argv[args+1][0]))
                    fprintf(stderr, "Error: no format value\n");
                else
                    data_format = atoi(argv[++args]);
                break;
            case 'I':                   // -I: number of input channels
                if (args>=argc-1 || !isdigit((int)argv[args+1][0]))
                    fprintf(stderr, "Error: no input chan count\n");
                else
                    in_chans = atoi(argv[++args]);
                break;
            case 'L':                   // -L: lockstep streaming
                lockstep = 1;
                break;
            case 'N':                   // -N: number of samples per block
                if (args>=argc-1 || !isdigit((int)argv[args+1][0]) ||
                    (sample_count = atoi(argv[++args])) < 1)
                    fprintf(stderr, "Error: no sample count\n");
                else if (sample_count > MAX_SAMPS)
                {
                    fprintf(stderr, "Error: maximum sample count %u\n", MAX_SAMPS);
                    sample_count = MAX_SAMPS;
                }
                break;
            case 'R':                   // -R: sample rate (samples/sec)
                if (args>=argc-1 || !isdigit((int)argv[args+1][0]))
                    fprintf(stderr, "Error: no sample rate\n");
                else if (sample_rate > MAX_SAMPLE_RATE)
                    fprintf(stderr, "Error: exceeded max sample rate\n");
                else
                    sample_rate = atoi(argv[++args]);
                break;
            case 'S':                   // -S: stream into named pipe (FIFO)
                if (args>=argc-1 || !argv[args+1][0])
                    fprintf(stderr, "Error: no FIFO name\n");
                else
                    fifo_name = argv[++args];
                break;
            case 'T':                   // -T: test mode
                testmode = 1;
                break;
            case 'V':                   // -V: verbose mode (display hex data)
                verbose = 1;
                break;
            default:
                printf("Error: unrecognised option '%s'\n", argv[args]);
                exit(1);
            }
        }
    }
    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE);
    signal(SIGINT, terminate);
    pwm_range = (PWM_FREQ * 2) / sample_rate;
    f = init_spi(SPI_FREQ);
    if (testmode)
    {
        printf("Testing %1.3f MHz SPI frequency: ", f/1e6);
        freq = test_spi_frequency(&vc_mem);
        printf("%7.3f MHz\n", freq);
        printf("Testing %5u Hz  PWM frequency: ", sample_rate);
        freq = test_pwm_frequency(&vc_mem);
        printf("%7.3f Hz\n", freq);
    }
    else if (sample_count == 0)
    {
        printf("SPI frequency %u Hz\n", f);
        val = adc_get_sample(0);
        printf("ADC value %u = %4.3fV\n", val, ADC_VOLTAGE(val));
    }
    else if (fifo_name)
    {
        if (create_fifo(fifo_name))
        {
            printf("Created FIFO '%s'\n", fifo_name);
            printf("Streaming %u samples per block at %u S/s %s\n",
                   sample_count, sample_rate, lockstep ? "(lockstep)" : "");
            adc_dma_init(&vc_mem, sample_count, 0);
            adc_stream_start();
            while (1)
                do_streaming(&vc_mem, stream_buff, STREAM_BUFFLEN, sample_count);
        }
    }
    else
    {
        printf("Reading %u samples at %u S/s\n", sample_count, sample_rate);
        adc_dma_init(&vc_mem, sample_count, 1);
        adc_stream_start();
        adc_stream_wait();
        adc_stream_stop();
        adc_stream_csv(&vc_mem, stream_buff, STREAM_BUFFLEN, sample_count);
        printf("%s", stream_buff);
    }
    terminate(0);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    terminate(0);
}

// Free memory & peripheral mapping and exit
void terminate(int sig)
{
    printf("Closing\n");
    spi_disable();
    stop_dma(DMA_CHAN_A);
    stop_dma(DMA_CHAN_B);
    stop_dma(DMA_CHAN_C);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&usec_regs);
    unmap_periph_mem(&pwm_regs);
    unmap_periph_mem(&clk_regs);
    unmap_periph_mem(&spi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    if (fifo_name)
        destroy_fifo(fifo_name, fifo_fd);
    if (samp_total)
        printf("Total samples %u, overruns %u\n", samp_total, overrun_total);
    exit(0);
}

// Map GPIO, DMA and SPI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&spi_regs, (void *)SPI0_BASE, PAGE_SIZE);
    map_periph(&clk_regs, (void *)CLK_BASE, PAGE_SIZE);
    map_periph(&pwm_regs, (void *)PWM_BASE, PAGE_SIZE);
    map_periph(&usec_regs, (void *)USEC_BASE, PAGE_SIZE);
}

// Get uncached memory
 void get_uncached_mem(MEM_MAP *mp, int size)
{
    if (!map_uncached_mem(mp, size))
        fail("Error: can't allocate uncached memory\n");
}

// Fetch single sample from ADC channel 0 or 1
int adc_get_sample(int chan)
{
    uint8_t txdata[ADC_RAW_LEN] = ADC_REQUEST(chan > 0);
    uint8_t rxdata[ADC_RAW_LEN];

    spi_cs(1);
    spi_xfer(txdata, rxdata, sizeof(txdata));
    spi_cs(0);
    if (verbose)
    {
        for (int i=0; i<ADC_RAW_LEN; i++)
            printf("%02X ", rxdata[i]);
        printf("\n");
    }
    return(ADC_RAW_VAL(*(uint16_t *)rxdata));
}

// Definitions for SPI frequency test
#define SPI_TEST_TI  (DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP  | DMA_CB_SRCE_INC)
#define TEST_NSAMPS  10

typedef struct {
    DMA_CB cbs[NUM_CBS];
    uint32_t txd[TEST_NSAMPS], val;
    volatile uint32_t usecs[2];
} TEST_DMA_DATA;

// Test SPI frequency
float test_spi_frequency(MEM_MAP *mp)
{
    TEST_DMA_DATA *dp=mp->virt;
    TEST_DMA_DATA dma_data = {
        .txd = {0,0,0,0,0,0,0,0,0,0}, .usecs = {0, 0},
        .cbs = {
        // Tx output: 2 initial transfers, then 10 timed transfers
            {SPI_TEST_TI, MEM(mp, dp->txd), REG(spi_regs, SPI_FIFO),           2*4, 0, CBS(1), 0}, // 0
            {SPI_TEST_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[0]),     4, 0, CBS(2), 0}, // 1
            {SPI_TEST_TI, MEM(mp, dp->txd), REG(spi_regs, SPI_FIFO), TEST_NSAMPS*4, 0, CBS(3), 0}, // 2
            {SPI_TEST_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[1]),     4, 0, 0,      0}, // 3
        }
    };
    memcpy(dp, &dma_data, sizeof(dma_data));                // Copy DMA data into uncached memory
    *REG32(spi_regs, SPI_DC) = (8<<24)|(1<<16)|(8<<8)|1;    // Set DMA priorities
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;                // Clear SPI FIFOs
    start_dma(mp, DMA_CHAN_A, &dp->cbs[0], 0);              // Start SPI Tx DMA
    *REG32(spi_regs, SPI_DLEN) = (TEST_NSAMPS + 2) * 4;     // Set data length, and SPI flags
    *REG32(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_DMA_EN;
    dma_wait(DMA_CHAN_A);                                   // Wait until complete
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;                // Clear accumulated Rx data
    return(dp->usecs[1] > dp->usecs[0] ?
           32.0 * TEST_NSAMPS / (dp->usecs[1] - dp->usecs[0]) : 0);
}

// Test PWM frequency
float test_pwm_frequency(MEM_MAP *mp)
{
    TEST_DMA_DATA *dp=mp->virt;
    TEST_DMA_DATA dma_data = {
        .val = pwm_range, .usecs = {0, 0},
        .cbs = {
        // Tx output: 2 initial transfers, then timed transfer
            {PWM_TI, MEM(mp, &dp->val),         REG(pwm_regs, PWM_FIF1), 4, 0, CBS(1), 0}, // 0
            {PWM_TI, MEM(mp, &dp->val),         REG(pwm_regs, PWM_FIF1), 4, 0, CBS(2), 0}, // 1
            {PWM_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[0]),  4, 0, CBS(3), 0}, // 2
            {PWM_TI, MEM(mp, &dp->val),         REG(pwm_regs, PWM_FIF1), 4, 0, CBS(4), 0}, // 3
            {PWM_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[1]),  4, 0, 0,      0}, // 4
        }
    };
    memcpy(dp, &dma_data, sizeof(dma_data));                // Copy DMA data into uncached memory
    *REG32(spi_regs, SPI_DC) = (8<<24)|(1<<16)|(8<<8)|1;    // Set DMA priorities
    init_pwm(PWM_FREQ, pwm_range, PWM_VALUE);               // Initialise PWM
    *REG32(pwm_regs, PWM_DMAC) = PWM_DMAC_ENAB | PWM_ENAB;  // Enable PWM DMA
    start_dma(mp, DMA_CHAN_A, &dp->cbs[0], 0);              // Start DMA
    start_pwm();                                            // Start PWM
    dma_wait(DMA_CHAN_A);                                   // Wait until complete
    stop_pwm();                                             // Stop PWM
    return(dp->usecs[1] > dp->usecs[0] ? 1e6 / (dp->usecs[1] - dp->usecs[0]) : 0);
}

typedef struct {
    DMA_CB cbs[NUM_CBS];
    uint32_t samp_size, pwm_val, adc_csd, txd[2];
    volatile uint32_t usecs[2], states[2], rxd1[MAX_SAMPS], rxd2[MAX_SAMPS];
} ADC_DMA_DATA;

// Initialise PWM-paced DMA for ADC sampling
void adc_dma_init(MEM_MAP *mp, int nsamp, int single)
{
    ADC_DMA_DATA *dp=mp->virt;
    ADC_DMA_DATA dma_data = {
        .samp_size = 2, .pwm_val = pwm_range, .txd={0xd0, in_chans>1 ? 0xf0 : 0xd0},
        .adc_csd = SPI_TFR_ACT | SPI_AUTO_CS | SPI_DMA_EN | SPI_FIFO_CLR | ADC_CE_NUM,
        .usecs = {0, 0}, .states = {0, 0}, .rxd1 = {0}, .rxd2 = {0},
        .cbs = {
        // Rx input: read data from usec clock and SPI, into 2 ping-pong buffers
            {SPI_RX_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[0]),  4, 0, CBS(1), 0}, // 0
            {SPI_RX_TI, REG(spi_regs, SPI_FIFO),   MEM(mp, dp->rxd1), nsamp*4, 0, CBS(2), 0}, // 1
            {SPI_RX_TI, REG(spi_regs, SPI_CS),     MEM(mp, &dp->states[0]), 4, 0, CBS(3), 0}, // 2
            {SPI_RX_TI, REG(usec_regs, USEC_TIME), MEM(mp, &dp->usecs[1]),  4, 0, CBS(4), 0}, // 3
            {SPI_RX_TI, REG(spi_regs, SPI_FIFO),   MEM(mp, dp->rxd2), nsamp*4, 0, CBS(5), 0}, // 4
            {SPI_RX_TI, REG(spi_regs, SPI_CS),     MEM(mp, &dp->states[1]), 4, 0, CBS(0), 0}, // 5
        // Tx output: 2 data writes to SPI for chan 0 & 1, or both chan 0
            {SPI_TX_TI, MEM(mp, dp->txd),          REG(spi_regs, SPI_FIFO), 8, 0, CBS(6), 0}, // 6
        // PWM ADC trigger: wait for PWM, set sample length, trigger SPI
            {PWM_TI,    MEM(mp, &dp->pwm_val),     REG(pwm_regs, PWM_FIF1), 4, 0, CBS(8), 0}, // 7
            {PWM_TI,    MEM(mp, &dp->samp_size),   REG(spi_regs, SPI_DLEN), 4, 0, CBS(9), 0}, // 8
            {PWM_TI,    MEM(mp, &dp->adc_csd),     REG(spi_regs, SPI_CS),   4, 0, CBS(7), 0}, // 9
        }
    };
    if (single)                                 // If single-shot, stop after first Rx block
        dma_data.cbs[2].next_cb = 0;
    memcpy(dp, &dma_data, sizeof(dma_data));    // Copy DMA data into uncached memory
    init_pwm(PWM_FREQ, pwm_range, PWM_VALUE);   // Initialise PWM, with DMA
    *REG32(pwm_regs, PWM_DMAC) = PWM_DMAC_ENAB | PWM_ENAB;
    *REG32(spi_regs, SPI_DC) = (8<<24) | (1<<16) | (8<<8) | 1;  // Set DMA priorities
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;                    // Clear SPI FIFOs
    start_dma(mp, DMA_CHAN_C, &dp->cbs[6], 0);  // Start SPI Tx DMA
    start_dma(mp, DMA_CHAN_B, &dp->cbs[0], 0);  // Start SPI Rx DMA
    start_dma(mp, DMA_CHAN_A, &dp->cbs[7], 0);  // Start PWM DMA, for SPI trigger
}

// Manage streaming output
void do_streaming(MEM_MAP *mp, char *vals, int maxlen, int nsamp)
{
    int n;

    if (!fifo_fd)
    {
        if ((fifo_fd = open_fifo_write(fifo_name)) > 0)
        {
            printf("Started streaming to FIFO '%s'\n", fifo_name);
            fifo_size = fifo_freespace(fifo_fd);
        }
    }
    if (fifo_fd)
    {
        if ((n=adc_stream_csv(mp, vals, maxlen, nsamp)) > 0)
        {
            if (!write_fifo(fifo_fd, vals, n))
            {
                printf("Stopped streaming\n");
                close(fifo_fd);
                fifo_fd = 0;
                usleep(100000);
            }
        }
        else
            usleep(10);
    }
}

// Start ADC data acquisition
void adc_stream_start(void)
{
    start_pwm();
}

// Wait until a (single) DMA cycle is complete
void adc_stream_wait(void)
{
    dma_wait(DMA_CHAN_B);
}

// Stop ADC data acquisition
void adc_stream_stop(void)
{
    stop_pwm();
}

// Fetch samples from ADC buffer, return comma-delimited integer values
// If in lockstep mode, discard new data if FIFO isn't empty
int adc_stream_csv(MEM_MAP *mp, char *vals, int maxlen, int nsamp)
{
    ADC_DMA_DATA *dp=mp->virt;
    uint32_t i, n, usec, slen=0;
    for (n=0; n<2 && slen==0; n++)
    {
        if (dp->states[n])
        {
            samp_total += nsamp;
            memcpy(rx_buff, n ? (void *)dp->rxd2 : (void *)dp->rxd1, nsamp*4);
            usec = dp->usecs[n];
            if (dp->states[n^1])
            {
                dp->states[0] = dp->states[1] = 0;
                overrun_total++;
                break;
            }
            dp->states[n] = 0;
            if (usec_start == 0)
                usec_start = usec;
            if (!lockstep || fifo_freespace(fifo_fd)>=fifo_size)
            {
                if (data_format == FMT_USEC)
                    slen += sprintf(&vals[slen], "%u", usec-usec_start);
                for (i=0; i<nsamp && slen+20<maxlen; i++)
                    slen += sprintf(&vals[slen], "%s%4.3f", slen ? "," : "",
                        ADC_VOLTAGE(ADC_RAW_VAL(rx_buff[i])));
                slen += sprintf(&vals[slen], "\n");
                if (verbose)
                {
                    for (int i=0; i<nsamp*4; i++)
                        printf("%02X ", *(((uint8_t *)rx_buff)+i));
                    printf("\n");
                }
            }
        }
    }
    vals[slen] = 0;
    return(slen);
}

// Test of SPI write cycles
// Redundant code, kept in as an explanation of SPI data length
int spi_tx_test(MEM_MAP *mp, uint16_t *buff, int nsamp)
{
    uint32_t n, a=0;

    nsamp = 8;
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
#if 1
    // Write data length to DLEN reg (with ACT clear)
    for (n=0; n<nsamp; n++)
    {
        *REG32(spi_regs, SPI_DLEN) = 2;
        *REG32(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_AUTO_CS | SPI_DMA_EN | SPI_FIFO_CLR;
        *REG32(spi_regs, SPI_FIFO) = n;
        usleep(5);
        a += *REG32(spi_regs, SPI_FIFO);
        // *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR; // Not needed, as ACT is already clear
    }
#else
    // Write data length to FIFO (with ACT set)
    *REG32(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_AUTO_CS | SPI_DMA_EN | SPI_FIFO_CLR;
    for (n=0; n<nsamp; n++)
    {
        *REG32(spi_regs, SPI_FIFO) = (2<<16) | SPI_TFR_ACT | SPI_FIFO_CLR;
        *REG32(spi_regs, SPI_FIFO) = n;
        usleep(5);
        a += *REG32(spi_regs, SPI_FIFO);
        // *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR; // Clearing ACT would disrupt comms
    }
#endif
    return(0);
}

// Wait until DMA is complete
void dma_wait(int chan)
{
    int n = 10000;

    do {
        usleep(10);
    } while (dma_transfer_len(chan) && --n);
    if (n == 0)
        printf("DMA transfer timeout\n");
}

// Create a FIFO (named pipe)
int create_fifo(char *fname)
{
    int ok=0;

    umask(0);
    if (mkfifo(fname, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH) < 0 && errno != EEXIST)
        printf("Can't open FIFO '%s'\n", fname);
    else
        ok = 1;
    return(ok);
}

// Open a FIFO for writing, return 0 if there is no reader
int open_fifo_write(char *fname)
{
    int f = open(fname, O_WRONLY | O_NONBLOCK);

    return(f == -1 ? 0 : f);
}

// Write to FIFO, return 0 if no reader
int write_fifo(int fd, void *data, int dlen)
{
    struct pollfd pollfd = {fd, POLLOUT, 0};

    poll(&pollfd, 1, 0);
    if (pollfd.revents&POLLOUT && !(pollfd.revents&POLLERR))
        return(fd ? write(fd, data, dlen) : 0);
    return(0);
}

// Return the free space in FIFO
uint32_t fifo_freespace(int fd)
{
    return(fcntl(fd, F_GETPIPE_SZ));
}

// Remove the FIFO
void destroy_fifo(char *fname, int fd)
{
    if (fd > 0)
        close(fd);
    unlink(fname);
}

// Initialise SPI0, given desired clock freq; return actual value
int init_spi(int hz)
{
    int f, div = (SPI_CLOCK_HZ / hz + 1) & ~1;

    gpio_set(SPI0_CE0_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_CE1_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_MISO_PIN, GPIO_ALT0, GPIO_PULLUP);
    gpio_set(SPI0_MOSI_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_SCLK_PIN, GPIO_ALT0, GPIO_NOPULL);
    while (div==0 || (f = SPI_CLOCK_HZ/div) > MAX_SPI_FREQ)
        div += 2;
    *REG32(spi_regs, SPI_CS) = 0x30;
    *REG32(spi_regs, SPI_CLK) = div;
    return(f);
}

// Clear SPI FIFOs
void spi_clear(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
}

// Set / clear SPI chip select
void spi_cs(int set)
{
    uint32_t csval = *REG32(spi_regs, SPI_CS);

    *REG32(spi_regs, SPI_CS) = set ? csval | 0x80 : csval & ~0x80;
}

// Transfer SPI bytes
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len)
{
    while (len--)
    {
        *REG8(spi_regs, SPI_FIFO) = *txd++;
        while((*REG32(spi_regs, SPI_CS) & (1<<17)) == 0) ;
        *rxd++ = *REG32(spi_regs, SPI_FIFO);
    }
}

// Disable SPI
void spi_disable(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
    *REG32(spi_regs, SPI_CS) = 0;
}

// Display SPI registers
void disp_spi(void)
{
    volatile uint32_t *p=REG32(spi_regs, SPI_CS);
    int i=0;

    while (spi_regstrs[i][0])
        printf("%-4s %08X ", spi_regstrs[i++], *p++);
    printf("\n");
}

// EOF

