/* Copyright (c) 2009 Axel Wachtler
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the name of the authors nor the names of its contributors
     may be used to endorse or promote products derived from this software
     without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE. */

/* $Id: $ */
/**
 * @file
 * @brief Interface for lwMesh
 * @ingroup grplwMesh
 */
#ifndef LWMESH_H
#define LWMESH_H

/**
 *  @addtogroup grplwMesh
 *  @{
 */
/* === includes ============================================================ */
#include "const.h"
#include "board.h"
#include "transceiver.h"
#include "radio.h"
#include "timer.h"

/* === macros ============================================================== */

/** number of buffers reserved for send, receive and route frames (min. 3) */
#define NWK_BUFFERS_AMOUNT         (3)


/** maximum number of endpoints (max. 4 bits) */
#define NWK_MAX_ENDPOINTS_AMOUNT    (0xF)

/** number of entries in the duplicate rejection table, Entries are never
 *  replaced before @ref NWK_DUPLICATE_REJECTION_TTL. So received frames are
 *  ignored and no frames can be sent if there are no free entries in this
 *  table. */
#define NWK_DUPLICATE_REJECTION_TABLE_SIZE (10)

/** Lifetime (in milliseconds) of elements in duplicate rejection table. Should
 *  be set at least to the maximum expected frame propagation time through the
 *  network multiplied by two. (TODO: add here: how long takes routing of one
 *  frame?)*/
#define NWK_DUPLICATE_REJECTION_TTL    (50)

/** timer interval for duplicate rejection table updates */
#define NWK_DUPLICATES_REJECTION_TIMER_INTERVAL (20)

/** number of entries in the routing table. Should be as large as the
  * expected number of nodes in the network.*/
#define NWK_ROUTE_TABLE_SIZE        (10)

/** time (in milliseconds) to wait for network acknowledgment after sending a
 *  frame with requested acknowledgment before @ref NWK_NO_ACK_STATUS is given.
 *   Should be set normally to the maximum expected frame propagation time
 *   through the network multiplied by two but can be adjusted in some cases */
#define NWK_ACK_WAIT_TIME  (50)

/** if defined, radio is set to sleep after transmission */
#ifndef NWK_SLEEPING_NODE
#define NWK_SLEEPING_NODE (0)
#endif

/** if defined routing support for this device is enabled, should be disabled
 *  for sleeping nodes */
#ifndef NWK_ENABLE_ROUTING
#define NWK_ENABLE_ROUTING (1)
#endif

/** for lightweight mesh control field, the first three bits are specified*/
#define NWK_NFCTL_MASK              (0x07)

/** endpoint ID for lightweight command frames. This endpoint is reserved and
 *  may not be used by any application. */
#define NWK_ENDPOINT_ID_LW_COMMAND  (0x0)

/** number of bytes including MAC and Lightweight Mesh headers
 * 9 MAC Bytes + 7 LW-Mesh Bytes + 2 CRC Bytes of MAC header*/
#define NWK_HEADER_SIZE             (9 + 7 + 2)

/** maximum application payload size (security disabled)
 * @ref MAX_FRAME_SIZE - @ref NWK_HEADER_SIZE */
#define NWK_MAX_PAYLOAD_SIZE        (MAX_FRAME_SIZE - NWK_HEADER_SIZE)

#define NWK_NO_TASK_ID              (255)

/** if defined, frames are sent and received via TX_ARET / RX_AACK mode with
 * 3 retransmission attempts, if not defined, basic TX / RX mode is used */
#define USE_RX_AACK                 (1)

/* === types =============================================================== */

/** options for @ref NWK_DataReq_t (OR operator "|" for multiple options) */
typedef enum SHORTENUM
{
    NWK_OPT_ACK_REQUEST         = 0x01, /**< request a network acknowledgment */
    NWK_OPT_ENABLE_SECURITY     = 0x02, /**< enable security (TODO: implement
                                             this) */
    NWK_OPT_LINK_LOCAL          = 0x04, /**< just send direct frame without
                                             routing */
    NWK_OPT_BROADCAST_PAN_ID    = 0x08, /**< set a broadcast pan ID */
} NWK_Opt_t;

/** options for @ref NWK_DataInd_t (OR operator "|" for multiple options) */
typedef enum SHORTENUM
{
    /**an acknowledgment for this frame was requested and will be sent by
     * stack */
    NWK_IND_OPT_ACK_REQUEST     = NWK_OPT_ACK_REQUEST,

    /** frame was sent with basic security enabled */
    NWK_IND_OPT_SECURED         = NWK_OPT_ENABLE_SECURITY,

    /** frame was received directly from source node without routing,
     *  link local @b was requested by source node */
    NWK_IND_OPT_LINK_LOCAL      = NWK_OPT_LINK_LOCAL,

    /** frame has a broadcast PAN ID */
    NWK_IND_OPT_BROADCAST_PAN_ID    = NWK_OPT_BROADCAST_PAN_ID,

    /** frame has a broadcast destination address */
    NWK_IND_OPT_BROADCAST       = 0x10,

    /** frame was received directly from source node without routing,
     *  link local @b was @b not requested by source node */
    NWK_IND_OPT_LOCAL           = 0x20,
} NWK_Ind_Opt_t;

/** status of @ref NWK_DataReq_t */
typedef enum SHORTENUM
{
    NWK_SUCCESS_STATUS,         /**< frame was sent successfully,
                                    acknowledgment frame was received, if
                                    requested */
    NWK_ERROR_STATUS,           /**< some unexpected error occurred while
                                    sending requested frame*/
    NWK_OUT_OF_MEMORY_STATUS,   /**< no buffers available, try calling
                                    lw_mesh_task_handler() to free some
                                    buffer*/
    NWK_NO_ACK_STATUS,          /**< frame was sent successfully, but no
                                    network ack could be received */
    NWK_PHY_CHANNEL_ACCESS_FAILURE_STATUS, /**< channel was not free while
                                            performing cca procedure */
    NWK_PHY_NO_ACK_STATUS,      /**< no phy ack was received (TX_AACK only)*/
} NWK_Stat_t;

/**
 * @brief structure for sending frames
 */
typedef struct NWK_DataReq_tag
{
    uint16_t    dstAddr;        /**< short destination address */
    uint8_t     dstEndpoint;    /**< destination endpoint (remote) */
    uint8_t     srcEndpoint;    /**< source endpoint (local) */
    NWK_Opt_t   options;        /**< data request options, see @ref NWK_Opt_t*/
    uint8_t     *data;          /**< pointer to the payload data */
    uint8_t     size;           /**< size of the payload data */
    void (*confirm)(struct NWK_DataReq_tag *);  /**< pointer to confirmation
                                                    callback function*/
    NWK_Stat_t  status;         /**< filled by stack, status of this Req */
    uint8_t     control;        /**< filled by stack, value of ack ctrl */
} NWK_DataReq_t;


/**
 * @brief structure for receiving frames
 */
typedef struct
{
    uint16_t    srcAddr;        /**< short source address */
    uint8_t     dstEndpoint;    /**< destination endpoint (local) */
    uint8_t     srcEndpoint;    /**< source endpoint (remote) */
    NWK_Ind_Opt_t   options;    /**< data request options, see
                                     @ref NWK_Ind_Opt_t */
    uint8_t     *data;          /**< pointer to the payload data */
    uint8_t     size;           /**< size of the payload data */
    uint8_t     lqi;            /**< LQI value reported by
                                     @ref usr_radio_receive_frame */
    uint8_t     rssi;           /**< RSSI value reported by
                                     @ref usr_radio_receive_frame */
} NWK_DataInd_t;

/*TODO: how to make this "packed" on 16 bit architectures? */
/**
 * @brief general lightweight mesh frame format
 */
typedef struct{
    uint16_t    m_fctl;     /**< MAC frame control field */
    uint8_t     m_seq;      /**< MAC sequence number */
    uint16_t    m_pid;      /**< MAC PAN ID */
    uint16_t    m_dstAddr;  /**< MAC destination address */
    uint16_t    m_srcAddr;  /**< MAC source address */

    uint8_t     lw_fctl;    /**< lightweight mesh frame control field */
    uint8_t     lw_seq;     /**< lightweight mesh sequence number */
    uint16_t    lw_srcAddr; /**< lightweight mesh source address */
    uint16_t    lw_dstAddr; /**< lightweight mesh destination address */
    uint8_t     lw_endpts;  /**< lightweight mesh src and dest endpoints */
    uint8_t     lw_payload[NWK_MAX_PAYLOAD_SIZE]; /**< application payload */

    uint16_t    m_crc;      /**< MAC check sum */
} NWK_FrameFormat_t;

typedef enum SHORTENUM
{
    NWK_TASK_NO_TASK,
    NWK_TASK_DATA_REQ,
    NWK_TASK_DATA_IND,
    NWK_TASK_TX_PENDING,
    NWK_TASK_ACK_PENDING,
} NWK_Task_t;

/**
 * @brief structure for task list to propagate @ref NWK_DataReq_t and
 * @ref NWK_DataInd_t to @ref lw_mesh_task_handler
 */
typedef struct{
    uint8_t             task_id;
    NWK_Task_t          type;
    timer_hdl_t         timer_id;
    NWK_FrameFormat_t   *frame; /**< Pointer to frame*/
    union{
        NWK_DataReq_t   req;    /**< data request structure */
        NWK_DataInd_t   ind;    /**< data indication structure*/
    } info;
} NWK_Tasklist_t;

/* === prototypes ========================================================== */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief network related ressource initialization
 *
 * @param pan_id own network identifier
 * @param short_addr own short address
 */
void lw_mesh_init(uint16_t pan_id, uint16_t short_addr, uint8_t channel);

/**
 * @brief network task handler. call as often as possible
 */
void lw_mesh_task_handler();

/**
 * @brief Frame transmission
 * 
 * Sends a network frame.
 *
 * @param req pointer to @ref NWK_DataReq_t with network frame information
 */
void lw_mesh_data_req(NWK_DataReq_t *req);

void lw_mesh_open_endpoint(uint8_t endpoint_id,
        bool (*data_ind_callback)(NWK_DataInd_t *));

void lw_mesh_set_ack_control(uint8_t control);

/** for call back from lw_mac interrupt function */
void lw_data_indication(NWK_FrameFormat_t *frame, uint8_t lw_payload_size,
        uint8_t lqi, uint8_t rssi);

void lw_tx_done(uint8_t task_id, radio_tx_done_t tx_status);

/* called by lightweight command when a network acknowledgment is received*/
void lw_ack_received(uint8_t lw_seq, uint8_t lw_cmd_ctrl_message);

#ifdef __cplusplus
} /* extern "C" */
#endif

/**
 *  @}
 */
#endif  /* #ifndef LWMESH_H */
