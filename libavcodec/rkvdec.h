/*
 * Copyright 2018 Rockchip Electronics Co., Ltd
 *     Author: James Lin<james.lin@rock-chips.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 *
 */

#ifndef AVCODEC_RKVDEC_H
#define AVCODEC_RKVDEC_H

#include "libavutil/hwcontext.h"
#include "libavutil/hwcontext_drm.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/internal.h"
#include "libavcodec/os_allocator.h"

#define LOG_LEVEL AV_LOG_ERROR

typedef unsigned int            RK_U32;
typedef signed int              RK_S32;
typedef signed short            RK_S16;
typedef unsigned char           RK_U8;
typedef unsigned short          RK_U16;
typedef signed long long int    RK_S64;

#define RKVDEC_IOC_MAGIC                       'l'

#define RKVDEC_IOC_SET_CLIENT_TYPE             _IOW(RKVDEC_IOC_MAGIC, 1, unsigned long)
#define RKVDEC_IOC_SET_CLIENT_TYPE_U32         _IOW(RKVDEC_IOC_MAGIC, 1, unsigned int)
#define RKVDEC_IOC_GET_HW_FUSE_STATUS          _IOW(RKVDEC_IOC_MAGIC, 2, unsigned long)
#define RKVDEC_IOC_SET_REG                     _IOW(RKVDEC_IOC_MAGIC, 3, unsigned long)
#define RKVDEC_IOC_GET_REG                     _IOW(RKVDEC_IOC_MAGIC, 4, unsigned long)
#define RKVDEC_IOC_PROBE_IOMMU_STATUS          _IOR(RKVDEC_IOC_MAGIC, 5, unsigned long)
#define RKVDEC_IOC_WRITE(nr, size)             _IOC(_IOC_WRITE, RKVDEC_IOC_MAGIC, (nr), (size))

#define ALIGN(value, x) ((value + (x - 1)) & (~(x - 1)))

#define RKVDEC_LOG(level, format,...) av_log(NULL, level,  "[%d|%s@%s,%d] " format "\n", level, __func__, __FILE__, __LINE__, ##__VA_ARGS__ )

typedef struct RKVDECPicEntry {
    union {
        struct {
            RK_U8   index_7bits : 7;
            RK_U8   flag : 1;
        };
        RK_U8   pic_entry;
    };
} RKVDECPicEntry;

typedef struct RKVDECPicture {
    RK_U8   valid;
    RK_U16  index;
    RK_U8   pic_struct;
    RK_U8   field_picture;
    RK_S32  field_poc[2];
    RK_S32  frame_idx;
    RK_U8   lt;
    RK_U8   reference;
} RKVDECPicture;

typedef struct RKVDECHwReq {
    RK_U32* req;
    RK_U32  size;
} RKVDECHwReq;

typedef struct RKVDECDevice {
    const char  *name;
    const char  *dev;
    RK_U32      priv_data_size;
    RK_S32      (*init)(void *ctx);
    RK_S32      (*prepare)(void *ctx, void* data, void* param);
    RK_S32      (*perform)(void *ctx);
    RK_S32      (*uninit)(void *ctx);
} RKVDECDevice;

typedef struct RKVDECContext{
    RKVDECDevice        dev;
    void                *dev_ctx;
    AVPacket            pkt;
    void*               pic_param;
    os_allocator        allocator;
    void                *allocator_ctx;
    AVBufferPool        *internal_pool;
    AVHWDeviceContext   *hwdc;
    AVHWFramesContext   *hwfc;    
    pthread_mutex_t     hwaccel_mutex;    
    RK_U8               err_info;
} RKVDECContext;

static inline void* ff_rkvdec_get_context(AVCodecContext *avctx)
{
    return avctx->internal->hwaccel_priv_data;
}

static inline int ff_rkvdec_get_dma_fd(AVFrame *f)
{
    if (f->format == AV_PIX_FMT_DRM_PRIME) {
        AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor*)f->data[0];
        if (desc->nb_objects > 0)
            return desc->objects[0].fd;
    }
    return f->linesize[2];
}

static inline int ff_rkvdec_get_dma_size(AVFrame *f)
{
    return f->linesize[0];
}

static inline uint8_t* ff_rkvdec_get_dma_ptr(AVFrame *f)
{
    return f->data[0];
}

static int get_rkvdec_picture_index2(RKVDECPicture DPB[], const RKVDECPicture* pic) {
    int i;
    if (pic->index) {
        for(i = 0; i < 16; i++) {
            if (DPB[i].valid && DPB[i].index == pic->index)
                return i;
        }
    }
    return -1;
}

#endif

