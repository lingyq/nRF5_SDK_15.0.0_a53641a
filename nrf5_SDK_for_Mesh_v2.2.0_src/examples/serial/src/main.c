/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "nrf_delay.h"
#include "boards.h"
#include "log.h"
#include "nrf_mesh_serial.h"
#include "mesh_app_utils.h"
#include "simple_hal.h"
#include "mesh_stack.h"
#include "mesh_softdevice_init.h"
#include "mesh_provisionee.h"
#include "nrf_mesh_config_examples.h"
#include "mesh_opt_prov.h"
#include "app_timer.h"

#define LED_BLINK_INTERVAL_SHORT_MS (100)
#define LED_BLINK_INTERVAL_MS       (200)
#define LED_BLINK_CNT_START         (2)

#define STATIC_AUTH_DATA {0x6E, 0x6F, 0x72, 0x64, 0x69, 0x63, 0x5F, 0x65, 0x78, 0x61, 0x6D, 0x70, 0x6C, 0x65, 0x5F, 0x31}

static bool m_device_provisioned;

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc     = DEV_BOARD_LF_CLK_CFG
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Enabling ECDH offloading...\n");
    ERROR_CHECK(mesh_opt_prov_ecdh_offloading_set(true));

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing serial interface...\n");
    ERROR_CHECK(nrf_mesh_serial_init(NULL));
}

static void initialize(void)
{
#if defined(NRF51) && defined(NRF_MESH_STACK_DEPTH)
    stack_depth_paint_stack();
#endif

    __LOG_INIT(LOG_MSK_DEFAULT | LOG_SRC_ACCESS | LOG_SRC_SERIAL | LOG_SRC_APP, LOG_LEVEL_INFO, log_callback_rtt);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Bluetooth Mesh Serial Interface Application -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

    nrf_clock_lf_cfg_t lfc_cfg = DEV_BOARD_LF_CLK_CFG;
    ERROR_CHECK(mesh_softdevice_init(lfc_cfg));
    mesh_init();

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initialization complete!\n");
}

static void start(void)
{
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data = static_auth_data,
            .prov_complete_cb = NULL,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    ERROR_CHECK(nrf_mesh_serial_enable());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Bluetooth Mesh Serial Interface Application started!\n");
}

int main(void)
{
    initialize();
    execution_start(start);
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
