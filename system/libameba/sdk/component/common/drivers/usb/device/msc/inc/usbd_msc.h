#ifndef USBD_MSC_H
#define USBD_MSC_H

#include <platform_opts.h>

#ifdef CONFIG_USBD_MSC

#include <platform/platform_stdlib.h>
#include "usb.h"
#include "usb_composite.h"
#include "osdep_service.h"
#include "sd.h"

/* MSC configurations */
#define USBD_MSC_RAM_DISK                    0       /* Use RAM as storage media, for test purpose only */

/* MSC Request Codes */
#define USBD_MSC_REQUEST_RESET               0xFF
#define USBD_MSC_REQUEST_GET_MAX_LUN         0xFE

#define USBD_MSC_MAX_LUN                     1
#define USBD_MSC_MAX_SCSI_CMD_SIZE           16
#define USBD_MSC_BLK_BITS                    9
#define USBD_MSC_BLK_SIZE                   (1 << USBD_MSC_BLK_BITS)

/* MSC Sub Classes */
#define USBD_MSC_SC_RBC                      1       /* Typically, flash devices */
#define USBD_MSC_SC_8020                     2       /* CD-ROM */
#define USBD_MSC_SC_QIC                      3       /* QIC-157 Tapes */
#define USBD_MSC_SC_UFI                      4       /* Floppy */
#define USBD_MSC_SC_8070                     5       /* Removable media */
#define USBD_MSC_SC_SCSI                     6       /* Transparent */

/* MSC Protocols */
#define USBD_MSC_PR_CB                       1       /* Control/Bulk w/o interrupt */
#define USBD_MSC_PR_CBI                      0       /* Control/Bulk/Interrupt */
#define USBD_MSC_PR_BULK                     0x50    /* bulk only */

#define USBD_MSC_BULK_OUT_EP_ADDRESS         0x02
#define USBD_MSC_BULK_IN_EP_ADDRESS          0x81

/* MSC thread priority*/
#define USBD_MSC_BULK_CMD_THREAD_PRIORITY   (tskIDLE_PRIORITY + 3) // Should be lower than USBD_IRQ_THREAD_PRIORITY

/* MSC device buffer configuration */
#define USBD_MSC_NBR_BUFHD                   2       /* Number of buffer header for bulk in/out data */
#define USBD_MSC_BUFLEN                     (32*512) /* Default size of buffer length */

/* MSC tasks stack size */
#define USBD_MSC_BULK_CMD_TASK_STACK_SIZE    512
#define USBD_MSC_BULK_DATA_TASK_STACK_SIZE   512

#define USBD_MSC_HALT_ERROR_TOLERANCE        20

/* RAM disk configurations */
#if USBD_MSC_RAM_DISK
#define USBD_MSC_RAM_DISK_SIZE              (USBD_MSC_BUFLEN * 2) // Should be >= 32KB
#define USBD_MSC_RAM_DISK_SECTORS           (USBD_MSC_RAM_DISK_SIZE >> USBD_MSC_BLK_BITS)
#endif

/* Debug options */
#define USBD_MSC_DEBUG                       0
#if USBD_MSC_DEBUG
#define USBD_MSC_INFO(fmt, args...)          printf("\n\r[MSC]%s: " fmt, __FUNCTION__, ## args)
#define USBD_MSC_WARN(fmt, args...)          printf("\n\r[MSC]%s: " fmt, __FUNCTION__, ## args)
#define USBD_MSC_ERROR(fmt, args...)         printf("\n\r[MSC]%s: " fmt, __FUNCTION__, ## args)
#define USBD_MSC_ENTER                       printf("\n\r[MSC]%s: =>", __FUNCTION__)
#define USBD_MSC_EXIT                        printf("\n\r[MSC]%s: <=", __FUNCTION__)
#define USBD_MSC_EXIT_ERR                    printf("\n\r[MSC]%s: ERR <=", __FUNCTION__)
#else
#define USBD_MSC_INFO(fmt, args...)
#define USBD_MSC_WARN(fmt, args...)
#define USBD_MSC_ERROR(fmt, args...)
#define USBD_MSC_ENTER
#define USBD_MSC_EXIT
#define USBD_MSC_EXIT_ERR
#endif

enum data_direction {
    DATA_DIR_UNKNOWN = 0,
    DATA_DIR_FROM_HOST,
    DATA_DIR_TO_HOST,
    DATA_DIR_NONE
};

//structure predefine
struct usb_msc_dev_t;
struct usb_msc_bufhd_t;

struct usb_msc_opts_t {
    SD_RESULT(*disk_init)(void);
    SD_RESULT(*disk_deinit)(void);
    SD_RESULT(*disk_getcapacity)(u32 *sectors);
    SD_RESULT(*disk_read)(u32 sector, u8 *buffer, u32 count);
    SD_RESULT(*disk_write)(u32 sector, const u8 *buffer, u32 count);
};

struct usb_msc_lun_t {
    unsigned int         initially_ro: 1;
    unsigned int         ro: 1;
    unsigned int         removable: 1;
    unsigned int         cdrom: 1;
    unsigned int         prevent_medium_removal: 1;
    unsigned int         registered: 1;
    unsigned int         info_valid: 1;
    unsigned int         nofua: 1;

    u32                  sense_data;
    u32                  sense_data_info;
    u32                  unit_attention_data;

    unsigned int         num_sectors;
    unsigned int         blkbits; /* bits of logical block size of bound block device */
    unsigned int         blksize; /* logical block size of bound block device */
    const char          *name;

    unsigned int         lba;     /* current read and write logical block address */
    u8                   is_open;
    _mutex               lun_mutex;
    
    struct usb_msc_opts_t     *lun_opts;
};


struct usb_msc_common_t {
    struct usb_msc_dev_t      *mscdev;

    struct usb_msc_lun_t     **luns;
    struct usb_msc_lun_t      *curlun;

    struct usb_gadget   *gadget;

    /* scsi cbw relevant */
    enum data_direction data_dir;
    u32                 data_size;
    u32                 data_size_from_cmnd;
    u32                 tag;
    u32                 residue;
    u8                  scsi_cmnd[USBD_MSC_MAX_SCSI_CMD_SIZE];
    u8                  cmnd_size;

    u8                  lun;    /* current lun*/
    u8                  nluns;

    u8                  nbufhd; /* number of buffer header */
    u8                  nbufhd_a;
    _list               bufhd_pool;
    _mutex              bufhd_mutex;
    
    /* bulk out cmd*/
    _list               boc_list;
    _mutex              boc_mutex;

    /* bolk out data*/
    _mutex              bod_mutex;
    _list               bod_list;
    
    struct usb_msc_bufhd_t   *curbh;  // current buffer header
    struct usb_msc_bufhd_t   *cbw_bh; // buffer header for CBW
    struct usb_msc_bufhd_t   *csw_bh; // buffer header for CSW

    unsigned int        can_stall: 1;
    unsigned int        phase_error: 1;
    unsigned int        short_packet_received: 1;
    unsigned int        bad_lun_okay: 1;
    unsigned int        running: 1;
};

typedef enum _bufhd_type {
    BUFHD_CBW = 0,
    BUFHD_CSW,
    BUFHD_DATA,
} usb_msc_bufhd_type;

struct usb_msc_bufhd_t {
    u8                 *prebuf;
    u8                 *buf;
    int                 buf_size;
    usb_msc_bufhd_type  type;
    _list               list;
    struct usb_request *reqin;     /* for bulkin responses */
    struct usb_request *reqout;
};

struct usb_msc_dev_t {
    struct usb_msc_common_t  *common;

    u16                 interface_number;

    struct usb_ep      *in_ep;
    struct usb_ep      *out_ep;
    unsigned int        bulk_in_enabled: 1;
    unsigned int        bulk_out_enabled: 1;

    struct task_struct  bulk_out_cmd_task;
};

int usbd_msc_lun_read(struct usb_msc_lun_t *curlun, u32 sector, u8 *buffer, u32 count);
int usbd_msc_lun_write(struct usb_msc_lun_t *curlun, u32 sector, const u8 *buffer, u32 count);
int usbd_msc_halt_bulk_in_endpoint(struct usb_msc_dev_t *mscdev);
void usbd_msc_put_bufhd(struct usb_msc_common_t *common, struct usb_msc_bufhd_t *bufhd);
struct usb_msc_bufhd_t *usbd_msc_get_bufhd(struct usb_msc_common_t *common);
int usbd_msc_bulk_in_transfer(struct usb_msc_dev_t *mscdev, struct usb_request *req);
int usbd_msc_bulk_out_transfer(struct usb_msc_dev_t *mscdev, struct usb_request *req);

#endif // CONFIG_USBD_MSC

#endif // USBD_MSC_H
