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

#ifndef AVCODEC_RKVDEC_DRM_H
#define AVCODEC_RKVDEC_DRM_H

#include <drm_fourcc.h>
#include "rkvdec.h"

typedef struct {
    AVFrame *buf;
    AVHWFramesContext *hwfc;
} RKVDECDRMBufferContext;

static void rkvdec_free_drm_buffer(void* opaque, uint8_t *data) {
    RKVDEC_LOG(LOG_LEVEL, "");
    AVDRMFrameDescriptor *desc = (AVDRMFrameDescriptor*)data;
    RKVDECDRMBufferContext *buf_ctx = (RKVDECDRMBufferContext*)opaque;
    RKVDECContext * const ctx = buf_ctx->hwfc->user_opaque;

    ctx->allocator.free(ctx->allocator_ctx, buf_ctx->buf);
    av_freep(&buf_ctx->buf);
    av_free(data);
}

static AVBufferRef* rkvdec_alloc_drm_buffer(void* opaque, RK_U32 size) {
    AVHWFramesContext * const hwfc = opaque;
    RKVDECContext * const ctx = hwfc->user_opaque;
    AVDRMFrameDescriptor *desc = av_mallocz(sizeof(AVDRMFrameDescriptor));
    AVDRMLayerDescriptor *layer = &desc->layers[0];
    AVFrame* buf = av_frame_alloc();
    RKVDECDRMBufferContext *buf_ctx = av_mallocz(sizeof(RKVDECDRMBufferContext));

    RKVDEC_LOG(LOG_LEVEL, "size:%d", size);

    buf->linesize[0] = size;
    ctx->allocator.alloc(ctx->allocator_ctx, buf);

    desc->nb_objects = 1;
    desc->objects[0].fd = ff_rkvdec_get_dma_fd(buf);
    desc->objects[0].size = size;

    desc->nb_layers = 1;
    layer->format = DRM_FORMAT_NV12;
    layer->nb_planes = 2;

    layer->planes[0].object_index = 0;
    layer->planes[0].offset = 0;
    layer->planes[0].pitch = hwfc->width;

    layer->planes[1].object_index = 0;
    layer->planes[1].offset = layer->planes[0].pitch * hwfc->height;
    layer->planes[1].pitch = layer->planes[0].pitch;

    buf_ctx->buf = buf;
    buf_ctx->hwfc = hwfc;

    return av_buffer_create(desc, sizeof(*desc), rkvdec_free_drm_buffer, 
                                buf_ctx, AV_BUFFER_FLAG_READONLY);
}

#endif

