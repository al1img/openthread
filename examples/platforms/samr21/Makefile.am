#
#  Copyright (c) 2017, The OpenThread Authors.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#  3. Neither the name of the copyright holder nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#

include $(abs_top_nlbuild_autotools_dir)/automake/pre.am

lib_LIBRARIES = libopenthread-samr21.a

override CFLAGS                              := $(filter-out -pedantic-errors,$(CFLAGS))
override CXXFLAGS                            := $(filter-out -pedantic-errors,$(CXXFLAGS))

libopenthread_samr21_a_CPPFLAGS                                                                    = \
    -D ARM_MATH_CM0PLUS=true                                                                         \
    -D BOARD=SAMR21_XPLAINED_PRO                                                                     \
    -D __SAMR21G18A__                                                                                \
    -I$(top_srcdir)/include                                                                          \
    -I$(top_srcdir)/examples/platforms                                                               \
    -I$(top_srcdir)/src/core                                                                         \
    -I$(top_srcdir)/third_party/microchip/common/boards                                              \
    -I$(top_srcdir)/third_party/microchip/common/utils                                               \
    -I$(top_srcdir)/third_party/microchip/sam0/boards                                                \
    -I$(top_srcdir)/third_party/microchip/sam0/boards/samr21_xplained_pro                            \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/port                                          \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system                                        \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/clock                                  \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/clock/clock_samd21_r21_da_ha1          \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/interrupt                              \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/interrupt/system_interrupt_samr21      \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/pinmux                                 \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/power                                  \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/power/power_sam_d_r_h                  \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/reset                                  \
    -I$(top_srcdir)/third_party/microchip/sam0/drivers/system/reset/reset_sam_d_r_h                  \
    -I$(top_srcdir)/third_party/microchip/sam0/utils                                                 \
    -I$(top_srcdir)/third_party/microchip/sam0/utils/cmsis/samr21/include                            \
    -I$(top_srcdir)/third_party/microchip/sam0/utils/cmsis/samr21/source                             \
    -I$(top_srcdir)/third_party/microchip/sam0/utils/header_files                                    \
    -I$(top_srcdir)/third_party/microchip/sam0/utils/preprocessor                                    \
    -I$(top_srcdir)/third_party/microchip/thirdparty/CMSIS/Include                                   \
    -Werror                                                                                          \
    -Wno-unused-parameter                                                                            \
    -Wno-implicit-function-declaration                                                               \
    $(NULL)

PLATFORM_SOURCES                                                                                   = \
    $(NULL)

libopenthread_samr21_a_SOURCES                                                                     = \
    $(NULL)

if OPENTHREAD_ENABLE_DIAG
PLATFORM_SOURCES                                                                                  += \
    $(NULL)
endif

libopenthread_samr21_a_SOURCES                                                                    += \
    $(PLATFORM_SOURCES)                                                                              \
    $(NULL)

nodist_libopenthread_samr21_a_SOURCES                                                             =  \
    @top_builddir@/third_party/microchip/common/utils/interrupt/interrupt_sam_nvic.c                 \
    @top_builddir@/third_party/microchip/sam0/boards/samr21_xplained_pro/board_init.c                \
    @top_builddir@/third_party/microchip/sam0/drivers/port/port.c                                    \
    @top_builddir@/third_party/microchip/sam0/drivers/system/clock/clock_samd21_r21_da_ha1/clock.c   \
    @top_builddir@/third_party/microchip/sam0/drivers/system/clock/clock_samd21_r21_da_ha1/gclk.c    \
    @top_builddir@/third_party/microchip/sam0/drivers/system/interrupt/system_interrupt.c            \
    @top_builddir@/third_party/microchip/sam0/drivers/system/pinmux/pinmux.c                         \
    @top_builddir@/third_party/microchip/sam0/drivers/system/system.c                                \
    @top_builddir@/third_party/microchip/sam0/utils/cmsis/samr21/source/gcc/startup_samr21.c         \
    @top_builddir@/third_party/microchip/sam0/utils/cmsis/samr21/source/system_samr21.c              \
    @top_builddir@/third_party/microchip/sam0/utils/syscalls/gcc/syscalls.c                          \
    $(NULL)

noinst_HEADERS                                                                                     = \
    $(NULL)

PRETTY_FILES                                                                                       = \
    $(PLATFORM_SOURCES)                                                                              \
    $(NULL)

Dash                                                                                               = -
libopenthread_samr21_a_LIBADD                                                                      = \
    $(shell find $(top_builddir)/examples/platforms/utils $(Dash)type f $(Dash)name "*.o")

include $(abs_top_nlbuild_autotools_dir)/automake/post.am
