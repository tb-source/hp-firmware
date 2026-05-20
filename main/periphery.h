/*
 * periphery.h
 *
 * Variant wrapper for LH/LG periphery definitions.
 */

#ifndef MAIN_PERIPHERY_H_
#define MAIN_PERIPHERY_H_

#include "sdkconfig.h"

#if CONFIG_PERIPHERY_VARIANT_LG
#include "periphery_lg.h"
#else
#include "periphery_lh.h"
#endif

#endif /* MAIN_PERIPHERY_H_ */