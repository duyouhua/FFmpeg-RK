/*
 * Copyright 2017 Rockchip Electronics Co., Ltd
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

#include "h264.h"
#include "h264dec.h"
#include "rkvdec_h264.h"
#include "rkvdec_drm.h"
#include "allocator_drm.h"
#include "hwaccel.h"
#include "decode.h"
#include "libavutil/time.h"
#include "libavutil/hwcontext_internal.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define LOG_LEVEL AV_LOG_DEBUG

static void rkvdec_h264_fill_picture(RKVDECPicture* dec_pic, const H264Picture *pic, int pic_structure) {
    dec_pic->valid = 1;
    dec_pic->index = ff_rkvdec_get_dma_fd(pic->f);
    dec_pic->pic_struct = pic_structure;
    dec_pic->lt = pic->long_ref;
    dec_pic->reference = pic->reference;
    dec_pic->frame_idx = pic->long_ref ? pic->pic_id : pic->frame_num;
    dec_pic->field_picture = pic->field_picture;
    dec_pic->field_poc[0] = 0;
    if (dec_pic->field_poc[0] != INT_MAX)
        dec_pic->field_poc[0] = pic->field_poc[0];
    dec_pic->field_poc[1] = 0;
    if (dec_pic->field_poc[1] != INT_MAX)
        dec_pic->field_poc[1] = pic->field_poc[1];
}

static void rkvdec_h264_init_picture(RKVDECPicture* dec_pic)
{
    dec_pic->valid = 0;
    dec_pic->field_poc[0] = dec_pic->field_poc[1] = 0;
}

static void rkvdec_h264_free_dmabuffer(void* opaque, uint8_t *data)
{
    RKVDECContext * const ctx = opaque;
    AVFrame* buf = (AVFrame*)data;
    RKVDEC_LOG(LOG_LEVEL, "size:%d", ff_rkvdec_get_dma_size(buf));

    if (!ctx || !buf)
        return;
    ctx->allocator.free(ctx->allocator_ctx, buf);
    av_freep(&buf);
}

static AVBufferRef* rkvdec_h264_alloc_dmabuffer(void* opaque, int size)
{
    RKVDECContext * const ctx = opaque;
    AVFrame* buf = av_frame_alloc();
    RKVDEC_LOG(LOG_LEVEL, "size:%d", size);

    buf->linesize[0] = size;
    ctx->allocator.alloc(ctx->allocator_ctx, buf);
    return av_buffer_create((uint8_t*)buf, size, rkvdec_h264_free_dmabuffer, ctx, 0);
}

static int rkvdec_h264_alloc_colmv(const H264Context* h)
{
    RKVDECContext * const ctx = ff_rkvdec_get_context(h->avctx);
    H264Picture *current_picture = h->cur_pic_ptr;

    if (current_picture->f->width >= 3840) {        
        AVBufferRef* prev_ref   = current_picture->hwaccel_priv_buf;
        const int b4_stride     = h->mb_width * 4 + 1;
        const int b4_array_size = b4_stride * h->mb_height * 4;
        const int colmv_size    = 2 * 2 * (b4_array_size + 4) * sizeof(int16_t);

        if (!ctx->internal_pool)
            ctx->internal_pool = av_buffer_pool_init2(colmv_size, ctx, rkvdec_h264_alloc_dmabuffer, NULL);

        current_picture->hwaccel_priv_buf = av_buffer_pool_get(ctx->internal_pool);
        current_picture->hwaccel_picture_private = current_picture->hwaccel_priv_buf->data;
        if (prev_ref)
            av_buffer_unref(&prev_ref);
    }

    return 0;
}

static int rkvdec_h264_start_frame(AVCodecContext          *avctx,
                                  av_unused const uint8_t *buffer,
                                  av_unused uint32_t       size)
{
    RKVDECContext * const ctx = ff_rkvdec_get_context(avctx);
    H264Context * const h = avctx->priv_data;
    const H264Picture *current_picture = h->cur_pic_ptr;
    const H264SliceContext *sl = &h->slice_ctx[0];
    const PPS *pps = h->ps.pps;
    const SPS *sps = h->ps.sps;
    RKVDECPicParamsH264 *pp = ctx->pic_param;
    int i, j;

    RKVDEC_LOG(LOG_LEVEL, "type:%d|struct:%d",
        sl->slice_type, h->picture_structure);

    pthread_mutex_lock(&ctx->hwaccel_mutex);

    memset(pp, 0, sizeof(RKVDECPicParamsH264));

    if (ctx->err_info) {
        if (h->avctx->active_thread_type & FF_THREAD_FRAME) {
            for (int i = 0; i < H264_MAX_PICTURE_COUNT; i++) {
                if (h->DPB[i].frame_num == h->poc.prev_frame_num) {
                    h->DPB[i].f->decode_error_flags |= FF_DECODE_ERROR_INVALID_BITSTREAM;
                    break;
                }
            }
        }
        ctx->err_info = 0;
    }

    rkvdec_h264_fill_picture(&pp->curr_pic, current_picture, h->picture_structure);

    rkvdec_h264_alloc_colmv(h);
    pp->curr_mv = ff_rkvdec_get_dma_fd(current_picture->hwaccel_picture_private);

    j = 0;
    for(i = 0; i < FF_ARRAY_ELEMS(pp->DPB); i++) {
        const H264Picture *r;
        if (j < h->short_ref_count) {
            r = h->short_ref[j++];
        } else {
            r = NULL;
            while (!r && j < h->short_ref_count + 16)
                r = h->long_ref[j++ - h->short_ref_count];
        }
        if (r && r->f && r->f->buf[0]) {
            rkvdec_h264_fill_picture(&pp->DPB[i], r, 0);
            pp->ref_colmv_list[i] = ff_rkvdec_get_dma_fd(r->hwaccel_picture_private);
        } else {
            rkvdec_h264_init_picture(&pp->DPB[i]);
            if (j - 1 < h->short_ref_count) {
                RKVDEC_LOG(LOG_LEVEL, "miss short ref");
                current_picture->f->decode_error_flags |= FF_DECODE_ERROR_MISSING_REFERENCE;
            }
        }
    }

    pp->frame_width                     = current_picture->f->width;
    pp->frame_height                    = current_picture->f->height;
    pp->frame_width_in_mbs_minus1       = h->mb_width  - 1;
    pp->frame_height_in_mbs_minus1      = h->mb_height / (2 - sps->frame_mbs_only_flag) - 1;
    pp->num_ref_frames                  = sps->ref_frame_count;

    pp->bit_fields  = ((h->picture_structure != PICT_FRAME) <<  0) |
                                        ((sps->mb_aff &&
                                        (h->picture_structure == PICT_FRAME)) <<  1) |
                                        (sps->residual_color_transform_flag   <<  2) |
                                        (0                                    <<  3) |
                                        (sps->chroma_format_idc               <<  4) |
                                        ((h->nal_ref_idc != 0)                <<  6) |
                                        (pps->constrained_intra_pred          <<  7) |
                                        (pps->weighted_pred                   <<  8) |
                                        (pps->weighted_bipred_idc             <<  9) |
                                        (1                                    << 11) |
                                        (sps->frame_mbs_only_flag             << 12) |
                                        (pps->transform_8x8_mode              << 13) |
                                        ((sps->level_idc >= 31)               << 14) |
                                        (1                                    << 15);

    pp->bit_depth_luma_minus8         = sps->bit_depth_luma - 8;
    pp->bit_depth_chroma_minus8       = sps->bit_depth_chroma - 8; 

    pp->pic_init_qs_minus26           = pps->init_qs - 26;
    pp->chroma_qp_index_offset        = pps->chroma_qp_index_offset[0];
    pp->second_chroma_qp_index_offset = pps->chroma_qp_index_offset[1];
    pp->pic_init_qp_minus26           = pps->init_qp - 26;
    pp->num_ref_idx_l0_active_minus1  = pps->ref_count[0] - 1;
    pp->num_ref_idx_l1_active_minus1  = pps->ref_count[1] - 1;

    pp->frame_num                     = h->poc.frame_num;
    pp->log2_max_frame_num_minus4     = sps->log2_max_frame_num - 4;
    pp->pic_order_cnt_type            = sps->poc_type;
    if (sps->poc_type == 0)
        pp->log2_max_pic_order_cnt_lsb_minus4 = sps->log2_max_poc_lsb - 4;
    else if (sps->poc_type == 1)
        pp->delta_pic_order_always_zero_flag = sps->delta_pic_order_always_zero_flag;
    pp->direct_8x8_inference_flag     = sps->direct_8x8_inference_flag;
    pp->entropy_coding_mode_flag      = pps->cabac;
    pp->pic_order_present_flag        = pps->pic_order_present;
    pp->num_slice_groups_minus1       = pps->slice_group_count - 1;
    pp->slice_group_map_type          = pps->mb_slice_group_map_type;
    pp->deblocking_filter_control_present_flag = pps->deblocking_filter_parameters_present;
    pp->redundant_pic_cnt_present_flag= pps->redundant_pic_cnt_present;

    pp->curr_view_id = 0;
    pp->curr_layer_id = 0;

    if (sps->scaling_matrix_present ||
        memcmp(sps->scaling_matrix4, pps->scaling_matrix4, sizeof(sps->scaling_matrix4)) ||
        memcmp(sps->scaling_matrix8, pps->scaling_matrix8, sizeof(sps->scaling_matrix8))) 
    {
        pp->scaleing_list_enable_flag = 1;
        memcpy(pp->scaling_lists4x4, pps->scaling_matrix4, sizeof(pp->scaling_lists4x4));
        memcpy(pp->scaling_lists8x8, pps->scaling_matrix8, sizeof(pp->scaling_lists8x8));
    }
    else
        pp->scaleing_list_enable_flag = 0;

    for (j = 0; j < 2; j++) {
        for (i = 0; i < 32; i++) {
            const H264Picture *r = sl->ref_list[j][i].parent;
            if (r && r->f && r->f->buf[0]) {
                rkvdec_h264_fill_picture(&pp->ref_frame_list[j][i], r, sl->ref_list[j][i].reference);
            }
            else {
                rkvdec_h264_init_picture(&pp->ref_frame_list[j][i]);
            }
        }
    }

    if (sl->slice_type_nos == AV_PICTURE_TYPE_P) {
        for (i = 0; i < 32 ; i++) {
            const H264Picture *r = sl->ref_list[0][i].parent;
            if (r && r->f && i < sl->ref_count[0])
                current_picture->f->decode_error_flags |= r->f->decode_error_flags;
        }
    }

    if (sl->slice_type_nos == AV_PICTURE_TYPE_B) {
        for (j = 0; j < 2; j++) {
            for (i = 0; i < 32 ; i++) {
                const H264Picture *r = sl->ref_list[j][i].parent;
                if (r && r->f && i < sl->ref_count[j])
                    current_picture->f->decode_error_flags |= r->f->decode_error_flags;
            }
        }
    }

    ctx->pkt.size = 0;

    return 0;
}

static int rkvdec_h264_end_frame(AVCodecContext *avctx)
{
    RKVDECContext * const ctx = ff_rkvdec_get_context(avctx);
    H264Context * const h = avctx->priv_data;
    const H264Picture *pic = h->cur_pic_ptr;
    int64_t begin;
    RKVDEC_LOG(LOG_LEVEL, "");

    if (ctx->pkt.size <= 0) {
        if (pic && pic->f)
            pic->f->decode_error_flags |= FF_DECODE_ERROR_INVALID_BITSTREAM;
        return 0;
    }

    ctx->dev.prepare(ctx->dev_ctx, &ctx->pkt, ctx->pic_param);

    begin = av_gettime();
    if (ctx->dev.perform(ctx->dev_ctx)) {
        pic->f->decode_error_flags |= FF_DECODE_ERROR_INVALID_BITSTREAM;
        ctx->err_info = pic->f->decode_error_flags;
    }

    RKVDEC_LOG(LOG_LEVEL, "err:%d|cost:%dns", ctx->err_info, av_gettime() - begin);
    pthread_mutex_unlock(&ctx->hwaccel_mutex);

    return 0;
}

static int rkvdec_h264_decode_slice(AVCodecContext *avctx,
                                   const uint8_t  *buffer,
                                   uint32_t        size)
{
    RKVDECContext * const ctx = ff_rkvdec_get_context(avctx);
    static const RK_U8 start_code[] = {0, 0, 1 }; 
    RKVDEC_LOG(LOG_LEVEL, "size:%d", size);

    if (ctx->pkt.buf->size - ctx->pkt.size < size + sizeof(start_code)) {
        av_buffer_realloc(&ctx->pkt.buf, ctx->pkt.size + size + 1024);
        ctx->pkt.data = ctx->pkt.buf->data;
    }

    memcpy(ctx->pkt.data + ctx->pkt.size, start_code, sizeof(start_code));
    ctx->pkt.size += sizeof(start_code);
    memcpy(ctx->pkt.data + ctx->pkt.size, buffer, size);
    ctx->pkt.size += size;

    return 0;
}

extern struct RKVDECDevice rkvdec341_h264;

static int rkvdec_h264_context_init(AVCodecContext *avctx)
{
    RKVDECContext * const ctx = ff_rkvdec_get_context(avctx);
    RKVDEC_LOG(LOG_LEVEL, "");

    pthread_mutex_init(&ctx->hwaccel_mutex, NULL);
    ctx->dev = rkvdec341_h264;
    ctx->dev_ctx = av_mallocz(ctx->dev.priv_data_size);
    ctx->dev.init(ctx->dev_ctx);

    ctx->allocator = allocator_drm;
    ctx->allocator.open(&ctx->allocator_ctx, 1);

    if (!avctx->hw_frames_ctx) 
        ff_decode_get_hw_frames_ctx(avctx, AV_HWDEVICE_TYPE_DRM);
    ((AVHWFramesContext*)avctx->hw_frames_ctx->data)->user_opaque = ctx;

    av_init_packet(&ctx->pkt);
    ctx->pkt.size = 0;
    ctx->pkt.buf = av_buffer_alloc(1024*1024);
    ctx->pkt.data = ctx->pkt.buf->data;
    ctx->pic_param = av_mallocz(sizeof(RKVDECPicParamsH264));

    RKVDEC_LOG(LOG_LEVEL, "dev:%s|allocator:%s", ctx->dev.name, ctx->allocator.name);

    return 0;
}

static int rkvdec_h264_context_uninit(AVCodecContext *avctx)
{
    RKVDECContext * const ctx = ff_rkvdec_get_context(avctx);
    RKVDEC_LOG(LOG_LEVEL, "");

    ctx->dev.uninit(ctx->dev_ctx);
    av_freep(&ctx->dev_ctx);
    av_freep(&ctx->pic_param);

    if (ctx->internal_pool)
        av_buffer_pool_uninit(&ctx->internal_pool);

    ctx->allocator.close(ctx->allocator_ctx);

    av_buffer_unref(&ctx->pkt.buf);

    pthread_mutex_destroy(&ctx->hwaccel_mutex);

    return 0;
}

static int rkvdec_h264_frame_params(AVCodecContext *avctx, AVBufferRef *hw_frames_ctx)
{
    RKVDECContext * const ctx = ff_rkvdec_get_context(avctx);    
    AVHWFramesContext *hw_frames = (AVHWFramesContext*)hw_frames_ctx->data;
    RKVDEC_LOG(LOG_LEVEL, "");
    hw_frames->format = AV_PIX_FMT_DRM_PRIME;
    hw_frames->sw_format = AV_PIX_FMT_NV12;
    hw_frames->width  = avctx->coded_width;
    hw_frames->height = avctx->coded_height;
    hw_frames->user_opaque = ctx;
    hw_frames->initial_pool_size = 0;
    hw_frames->internal->pool_internal = av_buffer_pool_init2(hw_frames->width * hw_frames->height * 2, hw_frames,
                                            rkvdec_alloc_drm_buffer, NULL);
    return 0;
}

const AVHWAccel ff_h264_rkvdec_hwaccel = {
    .name                 = "h264_rkvdec",
    .type                 = AVMEDIA_TYPE_VIDEO,
    .id                   = AV_CODEC_ID_H264,
    .pix_fmt              = AV_PIX_FMT_DRM_PRIME,
    .start_frame          = rkvdec_h264_start_frame,
    .end_frame            = rkvdec_h264_end_frame,
    .decode_slice         = rkvdec_h264_decode_slice,
    .init                 = rkvdec_h264_context_init,
    .uninit               = rkvdec_h264_context_uninit,
    .frame_params         = rkvdec_h264_frame_params,
    .priv_data_size       = sizeof(RKVDECContext),
    .frame_priv_data_size = sizeof(AVFrame),
    .caps_internal        = HWACCEL_CAP_ASYNC_SAFE,
};

