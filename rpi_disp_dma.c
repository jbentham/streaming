// Display active DMA channels on Raspberry Pi, see iosoft.blog for details
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
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "rpi_dma_utils.h"

// Number of DMA channels (excluding chan 15)
#define DMA_CHAN_COUNT  15

extern MEM_MAP dma_regs;

char used[100];

int main(int argc, char *argv[])
{
    int n, verbose=0;

    if (argc>1 && argv[1][0]=='-' && toupper(argv[1][1])=='V')
        verbose = 1;
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    for (n=0; n<DMA_CHAN_COUNT; n++)
    {
        if (verbose)
        {
            printf("Chan %u\n", n);
            disp_dma(n);
        }
        if (*REG32(dma_regs, DMA_REG(n, DMA_SRCE_AD)) ||
            *REG32(dma_regs, DMA_REG(n, DMA_DEST_AD)))
            sprintf(&used[strlen(used)], " %u", n);
    }
    printf("DMA channels in use:%s\n", used);
    unmap_periph_mem(&dma_regs);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
    exit(1);
}

// EOF
