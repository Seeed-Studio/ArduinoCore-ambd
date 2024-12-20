/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg/linux/drivers/dwc_otg_cil.h $
 * $Revision: #128 $
 * $Date: 2013/05/16 $
 * $Change: 2231774 $
 *
 * Synopsys HS OTG Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */

#if !defined(__DWC_CIL_H__)
#define __DWC_CIL_H__

#include "dwc_list.h"
#include "dwc_otg_regs.h"

#include "dwc_otg_core_if.h"
#include "dwc_otg_adp.h"
#include "ameba_otg.h"
#include "ameba_soc.h"
/**
 * @file
 * This file contains the interface to the Core Interface Layer.
 */

/** Macros defined for DWC OTG HW Release version */

#define OTG_CORE_REV_2_60a  0x4F54260A
#define OTG_CORE_REV_2_71a  0x4F54271A
#define OTG_CORE_REV_2_72a  0x4F54272A
#define OTG_CORE_REV_2_80a  0x4F54280A
#define OTG_CORE_REV_2_81a  0x4F54281A
#define OTG_CORE_REV_2_90a  0x4F54290A
#define OTG_CORE_REV_2_91a  0x4F54291A
#define OTG_CORE_REV_2_92a  0x4F54292A
#define OTG_CORE_REV_2_93a  0x4F54293A
#define OTG_CORE_REV_2_94a  0x4F54294A
#define OTG_CORE_REV_3_00a  0x4F54300A
#define OTG_CORE_REV_3_10a  0x4F54310A

/**
 * Information for each ISOC packet.
 */
typedef struct iso_pkt_info {
    uint32_t offset;
    uint32_t length;
    int32_t status;
} iso_pkt_info_t;

// for dwc_ep::type
#define DWC_OTG_EP_TYPE_CONTROL    0
#define DWC_OTG_EP_TYPE_ISOC       1
#define DWC_OTG_EP_TYPE_BULK       2
#define DWC_OTG_EP_TYPE_INTR       3

/**
 * The <code>dwc_ep</code> structure represents the state of a single
 * endpoint when acting in device mode. It contains the data items
 * needed for an endpoint to be activated and transfer packets.
 */
typedef struct dwc_ep {
    /** EP number used for register address lookup */
    uint8_t num;
    /** EP direction 0 = OUT */
    unsigned is_in: 1;
    /** EP active. */
    unsigned active: 1;

    /**
     * Periodic Tx FIFO # for IN EPs For INTR EP set to 0 to use non-periodic
     * Tx FIFO. If dedicated Tx FIFOs are enabled Tx FIFO # FOR IN EPs*/
    unsigned tx_fifo_num: 4;
    /** EP type: 0 - Control, 1 - ISOC,  2 - BULK,  3 - INTR */
    unsigned type: 2;

    /** DATA start PID for INTR and BULK EP */
    unsigned data_pid_start: 1;
    /** Frame (even/odd) for ISOC EP */
    unsigned even_odd_frame: 1;
    /** Max Packet bytes */
    unsigned maxpacket: 11;

    /** Max Transfer size */
    uint32_t maxxfer;

    /** @name Transfer state */
    /** @{ */

    /**
     * Pointer to the beginning of the transfer buffer -- do not modify
     * during transfer.
     */
    dwc_dma_t dma_addr;

    /* Flag indicating which one of two ISO desc chains currently is in use */
    uint8_t use_add_buf;

    uint8_t *start_xfer_buff;
    /** pointer to the transfer buffer */
    uint8_t *xfer_buff;
    /** Number of bytes to transfer */
    unsigned xfer_len: 19;
    /** Number of bytes transferred. */
    unsigned xfer_count: 19;
    /** Sent ZLP */
    unsigned sent_zlp: 1;
    /** Total len for control transfer */
    unsigned total_len: 19;

    /** stall clear flag */
    unsigned stall_clear_flag: 1;

    /** SETUP pkt cnt rollover flag for EP0 out*/
    unsigned stp_rollover;

    /** Max DMA Descriptor count for any EP */
#define MAX_DMA_DESC_CNT 256
    /** Allocated DMA Desc count */
    uint32_t desc_cnt;

    /** First ISO Desc in use in the first chain*/
    uint32_t iso_desc_first;
    /** Last ISO Desc in use in the second chain */
    uint32_t iso_desc_second;
    /** Flag indicated that iso transfers were started */
    uint8_t iso_transfer_started;

    /** bInterval */
    uint32_t bInterval;
    /** Next frame num to setup next ISOC transfer */
    uint32_t frame_num;
    /** Indicates SOF number overrun in DSTS */
    uint8_t frm_overrun;

#ifdef USBD_EN_ISOC
    /**
     * Variables specific for ISOC EPs
     *
     */
    /** DMA addresses of ISOC buffers */
    dwc_dma_t dma_addr0;
    dwc_dma_t dma_addr1;

    /** pointer to the transfer buffers */
    uint8_t *xfer_buff0;
    uint8_t *xfer_buff1;

    /** number of ISOC Buffer is processing */
    uint32_t proc_buf_num;
    /** Interval of ISOC Buffer processing */
    uint32_t buf_proc_intrvl;
    /** Data size for regular frame */
    uint32_t data_per_frame;

    uint32_t data_per_frame1;

    /* todo - pattern data support is to be implemented in the future */
    /** Data size for pattern frame */
    uint32_t data_pattern_frame;
    /** Frame number of pattern data */
    uint32_t sync_frame;

    /** bInterval */
    //uint32_t bInterval;
    /** ISO Packet number per frame */
    uint32_t pkt_per_frm;
    /** Next frame num for which will be setup DMA Desc */
    uint32_t next_frame;
    /** Number of packets per buffer processing */
    uint32_t pkt_cnt;
    /** Info for all isoc packets */
    iso_pkt_info_t *pkt_info;
    /** current pkt number */
    uint32_t cur_pkt;
    /** current pkt number */
    uint8_t *cur_pkt_addr;
    /** current pkt number */
    uint32_t cur_pkt_dma_addr;
#endif              /* USBD_EN_ISOC */

    /** @} */
} dwc_ep_t;

/**
 * The following parameters may be specified when starting the module. These
 * parameters define how the DWC_otg controller should be configured.
 */
typedef struct dwc_otg_core_params {
    /**
     * Specifies the OTG capabilities. The driver will automatically
     * detect the value for this parameter if none is specified.
     * 0 - HNP and SRP capable (default)
     * 1 - SRP Only capable
     * 2 - No HNP/SRP capable
     */
    int32_t otg_cap;

    /**
     * Specifies the maximum speed of operation in host and device mode.
     * The actual speed depends on the speed of the attached device and
     * the value of phy_type. The actual speed depends on the speed of the
     * attached device.
     * 0 - High Speed (default)
     * 1 - Full Speed
     */
    int32_t speed;

    /**
     * 0 - Use cC FIFO size parameters
     * 1 - Allow dynamic FIFO sizing (default)
     */
    int32_t enable_dynamic_fifo;

    /** Number of 4-byte words in the Rx FIFO in device mode when dynamic
     * FIFO sizing is enabled.
     * 16 to 32768 (default 1064)
     */
    int32_t dev_rx_fifo_size;

    /** Number of 4-byte words in the non-periodic Tx FIFO in device mode
     * when dynamic FIFO sizing is enabled.
     * 16 to 32768 (default 1024)
     */
    int32_t dev_nperio_tx_fifo_size;

    /** Number of 4-byte words in each of the periodic Tx FIFOs in device
     * mode when dynamic FIFO sizing is enabled.
     * 4 to 768 (default 256)
     */
    uint32_t dev_perio_tx_fifo_size[MAX_PERIO_FIFOS];

    /** The maximum transfer size supported in bytes.
     * 2047 to 65,535  (default 65,535)
     */
    int32_t max_transfer_size;

    /** The number of endpoints in addition to EP0 available for device
     * mode operations.
     * 1 to 15 (default 6 IN and OUT)
     * Note: The FPGA configuration supports a maximum of 6 IN and OUT
     * endpoints in addition to EP0.
     */
    int32_t dev_endpoints;

    int32_t ts_dline;

    /** Periodic Transfer Interrupt
     *  mode enable flag
     * 1 - Enabled
     * 0 - Disabled
     */
    int32_t pti_enable;

    /** HFIR Reload Control
     * 0 - The HFIR cannot be reloaded dynamically.
     * 1 - Allow dynamic reloading of the HFIR register during runtime.
     */
    int32_t reload_ctl;

    /** GAHBCFG: AHB Single Support
     * This bit when programmed supports SINGLE transfers for remainder
     * data in a transfer for DMA mode of operation.
     * 0 - in this case the remainder data will be sent using INCR burst size.
     * 1 - in this case the remainder data will be sent using SINGLE burst size.
     */
    int32_t ahb_single;

    /** Core Power down mode
     * 0 - No Power Down is enabled
     * 1 - Reserved
     * 2 - Complete Power Down (Hibernation)
     */
    int32_t power_down;

    /** OTG revision supported
     * 0 - OTG 1.3 revision
     * 1 - OTG 2.0 revision
     */
    int32_t otg_ver;

} dwc_otg_core_params_t;

typedef struct ep_xfer_info {
    struct dwc_otg_core_if *core_if;
    dwc_ep_t *ep;
    uint8_t state;
} ep_xfer_info_t;
/*
 * Device States
 */
typedef enum dwc_otg_lx_state {
    /** On state */
    DWC_OTG_L0,
    /** LPM sleep state*/
    DWC_OTG_L1,
    /** USB suspend state*/
    DWC_OTG_L2,
    /** Off state*/
    DWC_OTG_L3
} dwc_otg_lx_state_e;

struct dwc_otg_global_regs_backup {
    uint32_t gotgctl_local;
    uint32_t gintmsk_local;
    uint32_t gahbcfg_local;
    uint32_t gusbcfg_local;
    uint32_t grxfsiz_local;
    uint32_t gnptxfsiz_local;
#ifdef CONFIG_USB_DWC_OTG_LPM
    uint32_t glpmcfg_local;
#endif
    uint32_t gi2cctl_local;
    uint32_t hptxfsiz_local;
    uint32_t pcgcctl_local;
    uint32_t gdfifocfg_local;
    uint32_t dtxfsiz_local[MAX_EPS_CHANNELS];
    uint32_t gpwrdn_local;
    uint32_t xhib_pcgcctl;
    uint32_t xhib_gpwrdn;
};

struct dwc_otg_host_regs_backup {
    uint32_t hcfg_local;
    uint32_t haintmsk_local;
    uint32_t hcintmsk_local[MAX_EPS_CHANNELS];
    uint32_t hprt0_local;
    uint32_t hfir_local;
};

struct dwc_otg_dev_regs_backup {
    uint32_t dcfg;
    uint32_t dctl;
    uint32_t daintmsk;
    uint32_t diepmsk;
    uint32_t doepmsk;
    uint32_t diepctl[MAX_EPS_CHANNELS];
    uint32_t dieptsiz[MAX_EPS_CHANNELS];
    uint32_t diepdma[MAX_EPS_CHANNELS];
};
/**
 * The <code>dwc_otg_core_if</code> structure contains information needed to manage
 * the DWC_otg controller acting in either host or device mode. It
 * represents the programming view of the controller as a whole.
 */
struct dwc_otg_core_if {
    /** Parameters that define how the core should be configured.*/
    dwc_otg_core_params_t *core_params;

    /** Core Global registers starting at offset 000h. */
    dwc_otg_core_global_regs_t *core_global_regs;

    /** Device-specific information */
    dwc_otg_dev_if_t *dev_if;

    /** Value from SNPSID register */
    uint32_t snpsid;

    /*
     * Set to 1 if the core PHY interface bits in USBCFG have been
     * initialized.
     */
    uint8_t phy_init_done;

    /*
     * SRP Success flag, set by srp success interrupt in FS I2C mode
     */
    uint8_t srp_success;
    uint8_t srp_timer_started;
    /** Timer for SRP. If it expires before SRP is successful
     * clear the SRP. */
    dwc_timer_t *srp_timer;

#ifdef DWC_DEV_SRPCAP
    /* This timer is needed to power on the hibernated host core if SRP is not
     * initiated on connected SRP capable device for limited period of time
     */
    uint8_t pwron_timer_started;
    dwc_timer_t *pwron_timer;
#endif
    /* Common configuration information */
    /** Power and Clock Gating Control Register */
    volatile uint32_t *pcgcctl;
#define DWC_OTG_PCGCCTL_OFFSET 0xE00

    /** Push/pop addresses for endpoints or host channels.*/
    uint32_t *data_fifo[MAX_EPS_CHANNELS];
#define DWC_OTG_DATA_FIFO_OFFSET 0x1000
#define DWC_OTG_DATA_FIFO_SIZE 0x1000

    /** Total RAM for FIFOs (Bytes) */
    uint16_t total_fifo_size;
    /** Size of Rx FIFO (Bytes) */
    uint16_t rx_fifo_size;
    /** Size of Non-periodic Tx FIFO (Bytes) */
    uint16_t nperio_tx_fifo_size;

    /** 1 if PTI Enhancement mode is enabled, 0 otherwise. */
    uint8_t pti_enh_enable;

    /** Set to 1 if multiple packets of a high-bandwidth transfer is in
     * process of being queued */
    uint8_t queuing_high_bandwidth;

    /** Hardware Configuration -- stored here for convenience.*/
    hwcfg1_data_t hwcfg1;
    hwcfg2_data_t hwcfg2;
    hwcfg3_data_t hwcfg3;
    hwcfg4_data_t hwcfg4;
    fifosize_data_t hptxfsiz;

    /** Host and Device Configuration -- stored here for convenience.*/
    hcfg_data_t hcfg;
    dcfg_data_t dcfg;

    /** The operational State, during transations
     * (a_host>>a_peripherial and b_device=>b_host) this may not
     * match the core but allows the software to determine
     * transitions.
     */
    uint8_t op_state;

    /** Test mode for PET testing */
    uint8_t test_mode;

    /**
     * Set to 1 if the HCD needs to be restarted on a session request
     * interrupt. This is required if no connector ID status change has
     * occurred since the HCD was last disconnected.
     */
    uint8_t restart_hcd_on_session_req;

    /** HCD callbacks */
    /** A-Device is a_host */
#define A_HOST      (1)
    /** A-Device is a_suspend */
#define A_SUSPEND   (2)
    /** A-Device is a_peripherial */
#define A_PERIPHERAL    (3)
    /** B-Device is operating as a Peripheral. */
#define B_PERIPHERAL    (4)
    /** B-Device is operating as a Host. */
#define B_HOST      (5)

    /** HCD callbacks */
    struct dwc_otg_cil_callbacks *hcd_cb;
    /** PCD callbacks */
    struct dwc_otg_cil_callbacks *pcd_cb;

    /** Device mode Periodic Tx FIFO Mask */
    uint32_t p_tx_msk;
    /** Device mode Periodic Tx FIFO Mask */
    uint32_t tx_msk;

    /** Workqueue object used for handling several interrupts */
    //    dwc_workq_t *wq_otg;

    /** Timer object used for handling "Wakeup Detected" Interrupt */
    dwc_timer_t *wkp_timer;
    /** This arrays used for debug purposes for DEV OUT NAK enhancement */
    uint32_t start_doeptsiz_val[MAX_EPS_CHANNELS];
    ep_xfer_info_t ep_xfer_info[MAX_EPS_CHANNELS];
    dwc_timer_t *ep_xfer_timer[MAX_EPS_CHANNELS];

    /** Lx state of device */
    dwc_otg_lx_state_e lx_state;

    /** Saved Core Global registers */
    struct dwc_otg_global_regs_backup *gr_backup;
    /** Saved Device registers */
    struct dwc_otg_dev_regs_backup *dr_backup;

    /** Power Down Enable */
    uint32_t power_down;

    /** ADP support Enable */
    uint32_t adp_enable;

    /** ADP structure object */
    dwc_otg_adp_t adp;

    /** hibernation/suspend flag */
    int hibernation_suspend;

    /** Device mode extended hibernation flag */
    int xhib;

    /** OTG revision supported */
    uint32_t otg_ver;

    /** OTG status flag used for HNP polling */
    uint8_t otg_sts;

    /** Pointer to either hcd->lock or pcd->lock */
    dwc_spinlock_t *lock;

    /** Start predict NextEP based on Learning Queue if equal 1,
     * also used as counter of disabled NP IN EP's */
    uint8_t start_predict;

    /** NextEp sequence, including EP0: nextep_seq[] = EP if non-periodic and
     * active, 0xff otherwise */
    uint8_t nextep_seq[MAX_EPS_CHANNELS];

    /** Index of fisrt EP in nextep_seq array which should be re-enabled **/
    uint8_t first_in_nextep_seq;

    /** Frame number while entering to ISR - needed for ISOCs **/
    uint32_t frame_num;

    /** Flag to not perform ADP probing if IDSTS event happened */
    uint8_t stop_adpprb;

};

/*
 * This function is called when transfer is timed out on endpoint.
 */
extern void ep_xfer_timeout(void *ptr);

/*
 * The following functions are functions for works
 * using during handling some interrupts
 */
extern void w_conn_id_status_change(void *p);

extern void w_wakeup_detected(void *p);

/** Saves global register values into system memory. */
extern int dwc_otg_save_global_regs(dwc_otg_core_if_t *core_if);
/** Saves device register values into system memory. */
extern int dwc_otg_save_dev_regs(dwc_otg_core_if_t *core_if);
/** Restore global register values. */
extern int dwc_otg_restore_global_regs(dwc_otg_core_if_t *core_if);
/** Restore device register values. */
extern int dwc_otg_restore_dev_regs(dwc_otg_core_if_t *core_if, int rem_wakeup);
extern int restore_lpm_i2c_regs(dwc_otg_core_if_t *core_if);
extern int restore_essential_regs(dwc_otg_core_if_t *core_if, int rmode);
extern int dwc_otg_device_hibernation_restore(dwc_otg_core_if_t *core_if, int rem_wakeup, int reset);

/*
 * The following functions support initialization of the CIL driver component
 * and the DWC_otg controller.
 */
extern void dwc_otg_core_dev_init(dwc_otg_core_if_t *_core_if);

/** @name Device CIL Functions
 * The following functions support managing the DWC_otg controller in device
 * mode.
 */
/**@{*/
extern void dwc_otg_wakeup(dwc_otg_core_if_t *_core_if);
extern void dwc_otg_read_setup_packet(dwc_otg_core_if_t *_core_if, uint32_t *_dest);
extern uint32_t dwc_otg_get_frame_number(dwc_otg_core_if_t *_core_if);
extern void dwc_otg_ep0_activate(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_ep_activate(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_ep_deactivate(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_ep_start_transfer(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_ep_start_zl_transfer(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_ep0_start_transfer(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_ep0_continue_transfer(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_ep_write_packet(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep, int _dma);
extern void dwc_otg_ep_set_stall(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_ep_clear_stall(dwc_otg_core_if_t *_core_if, dwc_ep_t *_ep);
extern void dwc_otg_enable_device_interrupts(dwc_otg_core_if_t *_core_if);

#ifdef USBD_EN_ISOC
extern void dwc_otg_iso_ep_start_frm_transfer(dwc_otg_core_if_t *core_if, dwc_ep_t *ep);
extern void dwc_otg_iso_ep_start_buf_transfer(dwc_otg_core_if_t *core_if, dwc_ep_t *ep);
#endif /* USBD_EN_ISOC */
/**@}*/

extern int dwc_otg_check_haps_status(dwc_otg_core_if_t *core_if);

/**@}*/

/** @name Common CIL Functions
 * The following functions support managing the DWC_otg controller in either
 * device or host mode.
 */
/**@{*/

extern void dwc_otg_read_packet(dwc_otg_core_if_t *core_if,
    uint8_t *dest, uint16_t bytes);

extern void dwc_otg_flush_tx_fifo(dwc_otg_core_if_t *_core_if, const int _num);
extern void dwc_otg_flush_rx_fifo(dwc_otg_core_if_t *_core_if);
extern void dwc_otg_core_reset(dwc_otg_core_if_t *_core_if);

/**
 * This function returns the Core Interrupt register.
 */
static inline uint32_t dwc_otg_read_core_intr(dwc_otg_core_if_t *core_if)
{
    uint32_t v = DWC_READ_REG32(&core_if->core_global_regs->gintsts);
    v &= DWC_READ_REG32(&core_if->core_global_regs->gintmsk);
    return v;
}

/**
 * This function returns the OTG Interrupt register.
 */
static inline uint32_t dwc_otg_read_otg_intr(dwc_otg_core_if_t *core_if)
{
    return (DWC_READ_REG32(&core_if->core_global_regs->gotgint));
}

/**
 * This function reads the Device All Endpoints Interrupt register and
 * returns the IN endpoint interrupt bits.
 */
static inline uint32_t dwc_otg_read_dev_all_in_ep_intr(dwc_otg_core_if_t *
    core_if)
{
    uint32_t v = DWC_READ_REG32(&core_if->dev_if->dev_global_regs->daint);
    v &= DWC_READ_REG32(&core_if->dev_if->dev_global_regs->daintmsk);
    return (v & 0xffff);
}

/**
 * This function reads the Device All Endpoints Interrupt register and
 * returns the OUT endpoint interrupt bits.
 */
static inline uint32_t dwc_otg_read_dev_all_out_ep_intr(dwc_otg_core_if_t *
    core_if)
{
    uint32_t v = DWC_READ_REG32(&core_if->dev_if->dev_global_regs->daint);
    v &= DWC_READ_REG32(&core_if->dev_if->dev_global_regs->daintmsk);
    return ((v & 0xffff0000) >> 16);
}
/**
 * This function returns the Device IN EP Interrupt register
 */
static inline uint32_t dwc_otg_read_dev_in_ep_intr(dwc_otg_core_if_t *core_if,
    dwc_ep_t *ep)
{
    dwc_otg_dev_if_t *dev_if = core_if->dev_if;
    uint32_t v, msk, emp;
    msk = DWC_READ_REG32(&dev_if->dev_global_regs->diepmsk);
    emp = DWC_READ_REG32(&dev_if->dev_global_regs->dtknqr4_fifoemptymsk);
    msk |= ((emp >> ep->num) & 0x1) << 7;
    v = DWC_READ_REG32(&dev_if->in_ep_regs[ep->num]->diepint) & msk;
    return v;
}
/**
 * This function returns the Device OUT EP Interrupt register
 */
static inline uint32_t dwc_otg_read_dev_out_ep_intr(dwc_otg_core_if_t *
    core_if, dwc_ep_t *_ep)
{
    dwc_otg_dev_if_t *dev_if = core_if->dev_if;
    uint32_t v = 0;
    doepmsk_data_t msk = {.d32 = 0 };
    msk.d32 = DWC_READ_REG32(&dev_if->dev_global_regs->doepmsk);

    if (core_if->pti_enh_enable) {
        msk.b.pktdrpsts = 1;
    }

    v = DWC_READ_REG32(&dev_if->out_ep_regs[_ep->num]->doepint) & msk.d32;
    return v;
}

/**
 * This function returns the mode of the operation, host or device.
 *
 * @return 0 - Device Mode, 1 - Host Mode
 */
static inline uint32_t dwc_otg_mode(dwc_otg_core_if_t *_core_if)
{
    return (DWC_READ_REG32(&_core_if->core_global_regs->gintsts) & 0x1);
}

/**@}*/

/**
 * DWC_otg CIL callback structure. This structure allows the HCD and
 * PCD to register functions used for starting and stopping the PCD
 * and HCD for role change on for a DRD.
 */
typedef struct dwc_otg_cil_callbacks {
    /** Start function for role change */
    int (*start)(void *_p);
    /** Stop Function for role change */
    int (*stop)(void *_p);
    /** Disconnect Function for role change */
    int (*disconnect)(void *_p);
    /** Resume/Remote wakeup Function */
    int (*resume_wakeup)(void *_p);
    /** Suspend function */
    int (*suspend)(void *_p);
    /** Session Start (SRP) */
    int (*session_start)(void *_p);
#ifdef CONFIG_USB_DWC_OTG_LPM
    /** Sleep (switch to L0 state) */
    int (*sleep)(void *_p);
#endif
    /** Pointer passed to start() and stop() */
    void *p;
} dwc_otg_cil_callbacks_t;

extern void dwc_otg_cil_register_pcd_callbacks(dwc_otg_core_if_t *_core_if,
    dwc_otg_cil_callbacks_t *_cb,
    void *_p);
void dwc_otg_initiate_srp(void *core_if);

//////////////////////////////////////////////////////////////////////
/** Start the HCD.  Helper function for using the HCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_hcd_start(dwc_otg_core_if_t *core_if)
{
    if (core_if->hcd_cb && core_if->hcd_cb->start) {
        core_if->hcd_cb->start(core_if->hcd_cb->p);
    }
}

/** Stop the HCD.  Helper function for using the HCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_hcd_stop(dwc_otg_core_if_t *core_if)
{
    if (core_if->hcd_cb && core_if->hcd_cb->stop) {
        core_if->hcd_cb->stop(core_if->hcd_cb->p);
    }
}

/** Disconnect the HCD.  Helper function for using the HCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_hcd_disconnect(dwc_otg_core_if_t *core_if)
{
    if (core_if->hcd_cb && core_if->hcd_cb->disconnect) {
        core_if->hcd_cb->disconnect(core_if->hcd_cb->p);
    }
}

/** Inform the HCD the a New Session has begun.  Helper function for
 * using the HCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_hcd_session_start(dwc_otg_core_if_t *core_if)
{
    if (core_if->hcd_cb && core_if->hcd_cb->session_start) {
        core_if->hcd_cb->session_start(core_if->hcd_cb->p);
    }
}

#ifdef CONFIG_USB_DWC_OTG_LPM
/**
 * Inform the HCD about LPM sleep.
 * Helper function for using the HCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_hcd_sleep(dwc_otg_core_if_t *core_if)
{
    if (core_if->hcd_cb && core_if->hcd_cb->sleep) {
        core_if->hcd_cb->sleep(core_if->hcd_cb->p);
    }
}
#endif

/** Resume the HCD.  Helper function for using the HCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_hcd_resume(dwc_otg_core_if_t *core_if)
{
    if (core_if->hcd_cb && core_if->hcd_cb->resume_wakeup) {
        core_if->hcd_cb->resume_wakeup(core_if->hcd_cb->p);
    }
}

/** Start the PCD.  Helper function for using the PCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_pcd_start(dwc_otg_core_if_t *core_if)
{
    if (core_if->pcd_cb && core_if->pcd_cb->start) {
        core_if->pcd_cb->start(core_if->pcd_cb->p);
    }
}

/** Stop the PCD.  Helper function for using the PCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_pcd_stop(dwc_otg_core_if_t *core_if)
{
    if (core_if->pcd_cb && core_if->pcd_cb->stop) {
        core_if->pcd_cb->stop(core_if->pcd_cb->p);
    }
}

/** Suspend the PCD.  Helper function for using the PCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_pcd_suspend(dwc_otg_core_if_t *core_if)
{
    if (core_if->pcd_cb && core_if->pcd_cb->suspend) {
        core_if->pcd_cb->suspend(core_if->pcd_cb->p);
    }
}

/** Resume the PCD.  Helper function for using the PCD callbacks.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
static inline void cil_pcd_resume(dwc_otg_core_if_t *core_if)
{
    if (core_if->pcd_cb && core_if->pcd_cb->resume_wakeup) {
        core_if->pcd_cb->resume_wakeup(core_if->pcd_cb->p);
    }
}

//////////////////////////////////////////////////////////////////////

#endif
