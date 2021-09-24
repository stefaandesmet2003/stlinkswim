#ifndef __GENERAL_H
#define __GENERAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <inttypes.h>
#include <sys/types.h>

#if !defined(SYSTICKHZ)
# define SYSTICKHZ 1000
#endif
#define SYSTICKMS (1000 / SYSTICKHZ)

#endif

