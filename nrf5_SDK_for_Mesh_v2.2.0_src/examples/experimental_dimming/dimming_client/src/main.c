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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "generic_level_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "example_common.h"

#define APP_STATE_OFF                (0)
#define APP_STATE_ON                 (1)

#define APP_UNACK_MSG_REPEAT_COUNT   (2)

/* Timeout, in seconds, to demonstrate cumulative Delta Set messages with same TID value */
#define APP_TIMEOUT_FOR_TID_CHANGE   (3)

/* Client level parameter step size */
#define APP_LEVEL_STEP_SIZE          (10000L)

static generic_level_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool                   m_device_provisioned;

/* Forward declaration */
static void app_gen_level_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_level_client_status_cb(const generic_level_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_level_status_params_t * p_in);
static void app_gen_level_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);

const generic_level_client_callbacks_t client_cbs =
{
    .level_status_cb = app_generic_level_client_status_cb,
    .ack_transaction_status_cb = app_gen_level_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_level_client_publish_interval_cb
};

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_level_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_level_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

/* Generic Level client model interface: Process the received status message in this callback */
static void app_generic_level_client_status_cb(const generic_level_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_level_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Level server: 0x%04x, Present Level: %d, Target Level: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_level, p_in->target_level, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Level server: 0x%04x, Present Level: %d\n",
              p_meta->src.value, p_in->present_level);
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    static uint8_t client = 0;
    static generic_level_set_params_t set_params = {0};
    static generic_level_move_set_params_t move_set_params = {0};
    static generic_level_delta_set_params_t delta_set_params = {0};
    model_transition_t transition_params;
    static uint32_t timestamp = 0;
    timestamp_t current_time;

    /* Button 0,2,4: Reduce level, and send a Set/Delta Set/Move Set message
     * Button 1,3,5: Increase level, and send a Set/Delta Set/Move Set message
     *
     * Note: The cases for button number 4,5, and 6 are accessible only via RTT input.
     */

    switch(button_number)
    {
        case 0:
            set_params.level = (set_params.level - APP_LEVEL_STEP_SIZE) <= INT16_MIN ?
                               INT16_MIN : set_params.level - APP_LEVEL_STEP_SIZE;
            set_params.tid++;
            break;
        case 2:
            delta_set_params.delta_level = delta_set_params.delta_level - APP_LEVEL_STEP_SIZE;
            break;
        case 4:
            move_set_params.move_level = (move_set_params.move_level - APP_LEVEL_STEP_SIZE) <= INT16_MIN ?
                                         INT16_MIN : move_set_params.move_level - APP_LEVEL_STEP_SIZE;
            move_set_params.tid++;
            break;

        case 1:
            set_params.level = (set_params.level + APP_LEVEL_STEP_SIZE) >= INT16_MAX ?
                               INT16_MAX : set_params.level + APP_LEVEL_STEP_SIZE;
            set_params.tid++;
            break;
        case 3:
            delta_set_params.delta_level = delta_set_params.delta_level + APP_LEVEL_STEP_SIZE;
            break;
        case 5:
            move_set_params.move_level = (move_set_params.move_level + APP_LEVEL_STEP_SIZE) >= INT16_MAX ?
                                         INT16_MAX : move_set_params.move_level + APP_LEVEL_STEP_SIZE;
            move_set_params.tid++;
            break;
    }

    current_time = timer_now();
    if (timestamp + SEC_TO_US(APP_TIMEOUT_FOR_TID_CHANGE) < current_time)
    {
        delta_set_params.tid++;
    }
    timestamp = current_time;

    transition_params.delay_ms = APP_CONFIG_LEVEL_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_LEVEL_TRANSITION_TIME_MS;

    switch (button_number)
    {
        case 0:
        case 1:
            /* Demonstrate acknowledged transaction, using 1st client model instance */
            /* In this examples, users will not be blocked if the model is busy */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: Acknowledged Level Set: Target: %d Tid: %d Transition time: %d ms Delay: %d ms\n",
                  set_params.level, set_params.tid, transition_params.transition_time_ms, transition_params.delay_ms);
            (void)access_model_reliable_cancel(m_clients[client].model_handle);
            status = generic_level_client_set(&m_clients[client], &set_params, &transition_params);
            break;

        case 2:
        case 3:
            /* Demonstrate un-acknowledged transaction, using 2nd client model instance */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: UnAcknowledged Level Delta Set: Delta: %d Tid: %d Transition time: %d ms Delay: %d ms\n",
                  delta_set_params.delta_level, delta_set_params.tid, transition_params.transition_time_ms, transition_params.delay_ms);
            status = generic_level_client_delta_set_unack(&m_clients[client], &delta_set_params,
                                                          &transition_params, 0);
            break;

        /* Follwing cases are accessible only via RTT input */
        case 4:
        case 5:
            /* Demonstrate acknowledged transaction, using 1st client model instance */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: Acknowledged Level Move Set: Move: %d Tid: %d Transition time: %d ms Delay: %d ms\n",
                  move_set_params.move_level, move_set_params.tid, transition_params.transition_time_ms, transition_params.delay_ms);
            (void)access_model_reliable_cancel(m_clients[client].model_handle);
            status = generic_level_client_move_set(&m_clients[client], &move_set_params, &transition_params);
            break;

        /* Switch between each client instance */
        case 6:
            client++;
            client = (client < CLIENT_MODEL_INSTANCE_COUNT) ? client : 0;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Switching to client instance: %d\n", client);
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '6')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_CONFIG_MIC_SIZE;

        ERROR_CHECK(generic_level_client_init(&m_clients[i], i + 1));
    }
}

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE];
    uint8_t node_uuid_prefix[NODE_UUID_PREFIX_LEN] = LEVEL_CLIENT_NODE_UUID_PREFIX;

    ERROR_CHECK(mesh_app_uuid_gen(dev_uuid, node_uuid_prefix, NODE_UUID_PREFIX_LEN));
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    nrf_clock_lf_cfg_t lfc_cfg = DEV_BOARD_LF_CLK_CFG;
    ERROR_CHECK(mesh_softdevice_init(lfc_cfg));
    mesh_init();
}

static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .p_device_uri = NULL
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    execution_start(start);

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
