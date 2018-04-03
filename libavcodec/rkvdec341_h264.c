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
#include "libavutil/frame.h"
#include "libavutil/avassert.h"
#include "libavcodec/avcodec.h"
#include "libavcodec/internal.h"
#include "rkvdec_h264.h"
#include "put_bits64.h"
#include "allocator_drm.h"

#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define RKVDEC341H264_CABAC_TAB_SIZE        (3680*4 + 128)         /* bytes */
#define RKVDEC341H264_SPSPPS_SIZE           (256*32 + 128)         /* bytes */
#define RKVDEC341H264_RPS_SIZE              (128 + 128)            /* bytes */
#define RKVDEC341H264_SCALING_LIST_SIZE     (6*16 + 2*64 + 128)    /* bytes */
#define RKVDEC341H264_ERROR_INFO_SIZE       (256*144*4)            /* bytes */
#define RKVDEC341H264_DATA_SIZE             (2048 * 1024)          /* bytes */

typedef struct RKVDEC341RegH264 {
    struct {
        RK_U32    minor_ver : 8;
        RK_U32    level : 1;
        RK_U32    dec_support : 3;
        RK_U32    profile : 1;
        RK_U32    reserve0 : 1;
        RK_U32    codec_flag : 1;
        RK_U32    reserve1 : 1;
        RK_U32    prod_num : 16;
    } swreg0_id;

    struct {
        RK_U32    sw_dec_e : 1;
        RK_U32    sw_dec_clkgate_e : 1;
        RK_U32    reserve0 : 1;
        RK_U32    sw_timeout_mode : 1;
        RK_U32    sw_dec_irq_dis : 1;
        RK_U32    sw_dec_timeout_e : 1;
        RK_U32    sw_buf_empty_en : 1;
        RK_U32    sw_stmerror_waitdecfifo_empty : 1;
        RK_U32    sw_dec_irq : 1;
        RK_U32    sw_dec_irq_raw : 1;
        RK_U32    reserve2 : 2;
        RK_U32    sw_dec_rdy_sta : 1;
        RK_U32    sw_dec_bus_sta : 1;
        RK_U32    sw_dec_error_sta : 1;
        RK_U32    sw_dec_timeout_sta : 1;
        RK_U32    sw_dec_empty_sta : 1;
        RK_U32    sw_colmv_ref_error_sta : 1;
        RK_U32    sw_cabu_end_sta : 1;
        RK_U32    sw_h264orvp9_error_mode : 1;
        RK_U32    sw_softrst_en_p : 1;
        RK_U32    sw_force_softreset_valid : 1;
        RK_U32    sw_softreset_rdy : 1;
    } swreg1_int;

    struct {
        RK_U32    sw_in_endian : 1;
        RK_U32    sw_in_swap32_e : 1;
        RK_U32    sw_in_swap64_e : 1;
        RK_U32    sw_str_endian : 1;
        RK_U32    sw_str_swap32_e : 1;
        RK_U32    sw_str_swap64_e : 1;
        RK_U32    sw_out_endian : 1;
        RK_U32    sw_out_swap32_e : 1;
        RK_U32    sw_out_cbcr_swap : 1;
        RK_U32    reserve0 : 1;
        RK_U32    sw_rlc_mode_direct_write : 1;
        RK_U32    sw_rlc_mode : 1;
        RK_U32    sw_strm_start_bit : 7;
        RK_U32    reserve1 : 1;
        RK_U32    sw_dec_mode : 2;
        RK_U32    reserve2 : 2;
        RK_U32    sw_h264_rps_mode : 1;
        RK_U32    sw_h264_stream_mode : 1;
        RK_U32    sw_h264_stream_lastpacket : 1;
        RK_U32    sw_h264_firstslice_flag : 1;
        RK_U32    sw_h264_frame_orslice : 1;
        RK_U32    sw_buspr_slot_disable : 1;
        RK_U32    sw_colmv_mode : 1;
        RK_U32    sw_ycacherd_prior : 1;
    } swreg2_sysctrl;

    struct {
        RK_U32    sw_y_hor_virstride : 9;
        RK_U32    reserve : 2;
        RK_U32    sw_slice_num_highbit : 1;
        RK_U32    sw_uv_hor_virstride : 9;
        RK_U32    sw_slice_num_lowbits : 11;
    } swreg3_picpar;

    struct {
        RK_U32    sw_strm_rlc_base;
    } swreg4_strm_rlc_base;

    struct {
        RK_U32 sw_stream_len : 27;
    } swreg5_stream_rlc_len;

    struct {
        RK_U32    sw_cabactbl_base;
    } swreg6_cabactbl_prob_base;

    struct {
        RK_U32    sw_decout_base;
    } swreg7_decout_base;

    struct {
        RK_U32    sw_y_virstride : 20;
    } swreg8_y_virstride;

    struct {
        RK_U32    sw_yuv_virstride : 21;
    } swreg9_yuv_virstride;

    struct {
        RK_U32 sw_refer_base : 10;
        RK_U32 sw_ref_field : 1;
        RK_U32 sw_ref_topfield_used : 1;
        RK_U32 sw_ref_botfield_used : 1;
        RK_U32 sw_ref_colmv_use_flag : 1;

    } swreg10_24_refer0_14_base[15];

    RK_U32   swreg25_39_refer0_14_poc[15];

    struct {
        RK_U32 sw_cur_poc : 32;
    } swreg40_cur_poc;

    struct {
        RK_U32 sw_rlcwrite_base;
    } swreg41_rlcwrite_base;

    struct {
        RK_U32 sw_pps_base;
    } swreg42_pps_base;

    struct swreg_sw_rps_base {
        RK_U32 sw_rps_base;
    } swreg43_rps_base;

    struct swreg_strmd_error_e {
        RK_U32 sw_strmd_error_e : 28;
        RK_U32 reserve : 4;
    } swreg44_strmd_error_en;

    struct {
        RK_U32 sw_strmd_error_status : 28;
        RK_U32 sw_colmv_error_ref_picidx : 4;
    } swreg45_strmd_error_status;

    struct {
        RK_U32 sw_strmd_error_ctu_xoffset : 8;
        RK_U32 sw_strmd_error_ctu_yoffset : 8;
        RK_U32 sw_streamfifo_space2full : 7;
        RK_U32 reserve : 1;
        RK_U32 sw_vp9_error_ctu0_en : 1;
    } swreg46_strmd_error_ctu;

    struct {
        RK_U32 sw_saowr_xoffet : 9;
        RK_U32 reserve : 7;
        RK_U32 sw_saowr_yoffset : 10;
    } swreg47_sao_ctu_position;

    struct {
        RK_U32 sw_refer_base : 10;
        RK_U32 sw_ref_field : 1;
        RK_U32 sw_ref_topfield_used : 1;
        RK_U32 sw_ref_botfield_used : 1;
        RK_U32 sw_ref_colmv_use_flag : 1;

    } swreg48_refer15_base;

    RK_U32   swreg49_63_refer15_29_poc[15];

    struct {
        RK_U32 sw_performance_cycle : 32;
    } swreg64_performance_cycle;

    struct {
        RK_U32 sw_axi_ddr_rdata : 32;
    } swreg65_axi_ddr_rdata;

    struct {
        RK_U32 sw_axi_ddr_rdata : 32;
    } swreg66_axi_ddr_wdata;

    struct {
        union {
            struct {
                RK_U32 sw_busifd_resetn : 1;
                RK_U32 sw_cabac_resetn : 1;
                RK_U32 sw_dec_ctrl_resetn : 1;
                RK_U32 sw_transd_resetn : 1;
                RK_U32 sw_intra_resetn : 1;
                RK_U32 sw_inter_resetn : 1;
                RK_U32 sw_recon_resetn : 1;
                RK_U32 sw_filer_resetn : 1;
            };
            RK_U32 sw_resetn : 8;
        }
    } swreg67_fpgadebug_reset;

    struct {
        RK_U32 perf_cnt0_sel : 6;
        RK_U32 reserve0 : 2;
        RK_U32 perf_cnt1_sel : 6;
        RK_U32 reserve1 : 2;
        RK_U32 perf_cnt2_sel : 6;
    } swreg68_performance_sel;

    struct {
        RK_U32 perf_cnt0 : 32;
    } swreg69_performance_cnt0;

    struct {
        RK_U32 perf_cnt1 : 32;
    } swreg70_performance_cnt1;

    struct {
        RK_U32 perf_cnt2 : 32;
    } swreg71_performance_cnt2;

    RK_U32   swreg72_refer30_poc;
    RK_U32   swreg73_refer31_poc;

    struct {
        RK_U32 sw_h264_cur_poc1 : 32;
    } swreg74_h264_cur_poc1;

    struct {
        RK_U32 sw_errorinfo_base : 32;
    } swreg75_h264_errorinfo_base;

    struct {
        RK_U32 sw_slicedec_num : 14;
        RK_U32 reserve : 1;
        RK_U32 sw_strmd_detect_error_flag : 1;
        RK_U32 sw_error_packet_num : 14;
    } swreg76_h264_errorinfo_num;

    struct {
        RK_U32 sw_h264_error_en_highbits : 30;
        RK_U32 reserve : 2;
    } swreg77_h264_error_e;

    struct {
        RK_U32 sw_colmv_base;
    } swreg78_colmv_cur_base;

    struct {
        RK_U32 sw_colmv_base;
    } swreg79_94_colmv0_15_base[16];
}RKVDEC341RegH264;

static RK_U32 RKVDEC341_cabac_table[926 * 4] = {
    0x3602f114, 0xf1144a03, 0x4a033602, 0x68e97fe4, 0x36ff35fa, 0x21173307,
    0x00150217, 0x31000901, 0x390576db, 0x41f54ef3, 0x310c3e01, 0x321149fc,
    0x2b094012, 0x431a001d, 0x68095a10, 0x68ec7fd2, 0x4ef34301, 0x3e0141f5,
    0x5fef56fa, 0x2d093dfa, 0x51fa45fd, 0x370660f5, 0x56fb4307, 0x3a005802,
    0x5ef64cfd, 0x45043605, 0x580051fd, 0x4afb43f9, 0x50fb4afc, 0x3a0148f9,
    0x3f002900, 0x3f003f00, 0x560453f7, 0x48f96100, 0x3e03290d, 0x4efc2d00,
    0x7ee560fd, 0x65e762e4, 0x52e443e9, 0x53f05eec, 0x5beb6eea, 0x5df366ee,
    0x5cf97fe3, 0x60f959fb, 0x2efd6cf3, 0x39ff41ff, 0x4afd5df7, 0x57f85cf7,
    0x36057ee9, 0x3b063c06, 0x30ff4506, 0x45fc4400, 0x55fe58f8, 0x4bff4efa,
    0x36024df9, 0x44fd3205, 0x2a063201, 0x3f0151fc, 0x430046fc, 0x4cfe3902,
    0x4004230b, 0x230b3d01, 0x180c1912, 0x240d1d0d, 0x49f95df6, 0x2e0d49fe,
    0x64f93109, 0x35023509, 0x3dfe3505, 0x38003800, 0x3cfb3ff3, 0x39043eff,
    0x390445fa, 0x3304270e, 0x4003440d, 0x3f093d01, 0x27103207, 0x34042c05,
    0x3cfb300b, 0x3b003bff, 0x2c052116, 0x4eff2b0e, 0x45093c00, 0x28021c0b,
    0x31002c03, 0x2c022e00, 0x2f003302, 0x3e022704, 0x36002e06, 0x3a023603,
    0x33063f04, 0x35073906, 0x37063406, 0x240e2d0b, 0x52ff3508, 0x4efd3707,
    0x1f162e0f, 0x071954ff, 0x031cf91e, 0x0020041c, 0x061eff22, 0x0920061e,
    0x1b1a131f, 0x14251e1a, 0x4611221c, 0x3b054301, 0x1e104309, 0x23122012,
    0x1f181d16, 0x2b122617, 0x3f0b2914, 0x40093b09, 0x59fe5eff, 0x4cfa6cf7,
    0x2d002cfe, 0x40fd3400, 0x46fc3bfe, 0x52f84bfc, 0x4df766ef, 0x2a001803,
    0x37003000, 0x47f93bfa, 0x57f553f4, 0x3a0177e2, 0x24ff1dfd, 0x2b022601,
    0x3a0037fa, 0x4afd4000, 0x46005af6, 0x1f051dfc, 0x3b012a07, 0x48fd3afe,
    0x61f551fd, 0x05083a00, 0x120e0e0a, 0x28021b0d, 0x46fd3a00, 0x55f84ffa,
    0x6af30000, 0x57f66af0, 0x6eee72eb, 0x6eea62f2, 0x67ee6aeb, 0x6ce96beb,
    0x60f670e6, 0x5bfb5ff4, 0x5eea5df7, 0x430956fb, 0x55f650fc, 0x3c0746ff,
    0x3d053a09, 0x320f320c, 0x36113112, 0x2e07290a, 0x310733ff, 0x29093408,
    0x37022f06, 0x2c0a290d, 0x35053206, 0x3f04310d, 0x45fe4006, 0x46063bfe,
    0x1f092c0a, 0x35032b0c, 0x260a220e, 0x280d34fd, 0x2c072011, 0x320d2607,
    0x2b1a390a, 0x0e0b0b0e, 0x0b120b09, 0xfe170915, 0xf120f120, 0xe927eb22,
    0xe129df2a, 0xf426e42e, 0xe82d1d15, 0xe630d335, 0xed2bd541, 0x091ef627,
    0x1b141a12, 0x52f23900, 0x61ed4bfb, 0x001b7ddd, 0xfc1f001c, 0x0822061b,
    0x16180a1e, 0x20161321, 0x29151f1a, 0x2f172c1a, 0x470e4110, 0x3f063c08,
    0x18154111, 0x171a1417, 0x171c201b, 0x2817181c, 0x1d1c2018, 0x39132a17,
    0x3d163516, 0x280c560b, 0x3b0e330b, 0x47f94ffc, 0x46f745fb, 0x44f642f8,
    0x45f449ed, 0x43f146f0, 0x46ed3eec, 0x41ea42f0, 0xfe093fec, 0xf721f71a,
    0xfe29f927, 0x0931032d, 0x3b241b2d, 0x23f942fa, 0x2df82af9, 0x38f430fb,
    0x3efb3cfa, 0x4cf842f8, 0x51fa55fb, 0x51f94df6, 0x49ee50ef, 0x53f64afc,
    0x43f747f7, 0x42f83dff, 0x3b0042f2, 0xf3153b02, 0xf927f221, 0x0233fe2e,
    0x113d063c, 0x3e2a2237, 0x00000000, 0x00000000, 0x3602f114, 0xf1144a03,
    0x4a033602, 0x68e97fe4, 0x36ff35fa, 0x19163307, 0x00100022, 0x290409fe,
    0x410276e3, 0x4ff347fa, 0x32093405, 0x360a46fd, 0x1613221a, 0x02390028,
    0x451a2429, 0x65f17fd3, 0x47fa4cfc, 0x34054ff3, 0x5af34506, 0x2b083400,
    0x52fb45fe, 0x3b0260f6, 0x57fd4b02, 0x380164fd, 0x55fa4afd, 0x51fd3b00,
    0x5ffb56f9, 0x4dff42ff, 0x56fe4601, 0x3d0048fb, 0x3f002900, 0x3f003f00,
    0x560453f7, 0x48f96100, 0x3e03290d, 0x33070f0d, 0x7fd95002, 0x60ef5bee,
    0x62dd51e6, 0x61e966e8, 0x63e877e5, 0x66ee6eeb, 0x50007fdc, 0x5ef959fb,
    0x27005cfc, 0x54f14100, 0x49fe7fdd, 0x5bf768f4, 0x37037fe1, 0x37073807,
    0x35fd3d08, 0x4af94400, 0x67f358f7, 0x59f75bf3, 0x4cf85cf2, 0x6ee957f4,
    0x4ef669e8, 0x63ef70ec, 0x7fba7fb2, 0x7fd27fce, 0x4efb42fc, 0x48f847fc,
    0x37ff3b02, 0x4bfa46f9, 0x77de59f8, 0x14204bfd, 0x7fd4161e, 0x3dfb3600,
    0x3cff3a00, 0x43f83dfd, 0x4af254e7, 0x340541fb, 0x3d003902, 0x46f545f7,
    0x47fc3712, 0x3d073a00, 0x19122909, 0x2b052009, 0x2c002f09, 0x2e023300,
    0x42fc2613, 0x2a0c260f, 0x59002209, 0x1c0a2d04, 0xf5211f0a, 0x0f12d534,
    0xea23001c, 0x0022e726, 0xf420ee27, 0x0000a266, 0xfc21f138, 0xfb250a1d,
    0xf727e333, 0xc645de34, 0xfb2cc143, 0xe3370720, 0x00000120, 0xe721241b,
    0xe424e222, 0xe526e426, 0xf023ee22, 0xf820f222, 0x0023fa25, 0x121c0a1e,
    0x291d191a, 0x48024b00, 0x230e4d08, 0x23111f12, 0x2d111e15, 0x2d122a14,
    0x36101a1b, 0x38104207, 0x430a490b, 0x70e974f6, 0x3df947f1, 0x42fb3500,
    0x50f74df5, 0x57f654f7, 0x65eb7fde, 0x35fb27fd, 0x4bf53df9, 0x5bef4df1,
    0x6fe76be7, 0x4cf57ae4, 0x34f62cf6, 0x3af739f6, 0x45f948f0, 0x4afb45fc,
    0x420256f7, 0x200122f7, 0x34051f0b, 0x43fe37fe, 0x59f84900, 0x04073403,
    0x0811080a, 0x25031310, 0x49fb3dff, 0x4efc46ff, 0x7eeb0000, 0x6eec7ce9,
    0x7ce77ee6, 0x79e569ef, 0x66ef75e5, 0x74e575e6, 0x5ff67adf, 0x5ff864f2,
    0x72e46fef, 0x50fe59fa, 0x55f752fc, 0x48ff51f8, 0x43014005, 0x45003809,
    0x45074501, 0x43fa45f9, 0x40fe4df0, 0x43fa3d02, 0x390240fd, 0x42fd41fd,
    0x33093e00, 0x47fe42ff, 0x46ff4bfe, 0x3c0e48f7, 0x2f002510, 0x250b2312,
    0x290a290c, 0x290c3002, 0x3b00290d, 0x28133203, 0x32124203, 0xfa12fa13,
    0xf41a000e, 0xe721f01f, 0xe425ea21, 0xe22ae227, 0xdc2dd62f, 0xef29de31,
    0xb9450920, 0xc042c13f, 0xd936b64d, 0xf629dd34, 0xff280024, 0x1a1c0e1e,
    0x370c2517, 0xdf25410b, 0xdb28dc27, 0xdf2ee226, 0xe828e22a, 0xf426e331,
    0xfd26f628, 0x141ffb2e, 0x2c191e1d, 0x310b300c, 0x16162d1a, 0x151b1617,
    0x1c1a1421, 0x221b181e, 0x27192a12, 0x460c3212, 0x470e3615, 0x2019530b,
    0x36153115, 0x51fa55fb, 0x51f94df6, 0x49ee50ef, 0x53f64afc, 0x43f747f7,
    0x42f83dff, 0x3b0042f2, 0xf6113b02, 0xf72af320, 0x0035fb31, 0x0a440340,
    0x392f1b42, 0x180047fb, 0x2afe24ff, 0x39f734fe, 0x41fc3ffa, 0x52f943fc,
    0x4cfd51fd, 0x4efa48f9, 0x44f248f4, 0x4cfa46fd, 0x3efb42fb, 0x3dfc3900,
    0x36013cf7, 0xf6113a02, 0xf72af320, 0x0035fb31, 0x0a440340, 0x392f1b42,
    0x00000000, 0x00000000, 0x3602f114, 0xf1144a03, 0x4a033602, 0x68e97fe4,
    0x36ff35fa, 0x101d3307, 0x000e0019, 0x3efd33f6, 0x101a63e5, 0x66e855fc,
    0x39063905, 0x390e49ef, 0x0a142814, 0x0036001d, 0x610c2a25, 0x75ea7fe0,
    0x55fc4afe, 0x390566e8, 0x58f25dfa, 0x37042cfa, 0x67f159f5, 0x391374eb,
    0x54043a14, 0x3f016006, 0x6af355fb, 0x4b063f05, 0x65ff5afd, 0x4ffc3703,
    0x61f44bfe, 0x3c0132f9, 0x3f002900, 0x3f003f00, 0x560453f7, 0x48f96100,
    0x3e03290d, 0x58f72207, 0x7fdc7fec, 0x5ff25bef, 0x56e754e7, 0x5bef59f4,
    0x4cf27fe1, 0x5af367ee, 0x500b7fdb, 0x54024c05, 0x37fa4e05, 0x53f23d04,
    0x4ffb7fdb, 0x5bf568f5, 0x41007fe2, 0x48004ffe, 0x38fa5cfc, 0x47f84403,
    0x56fc62f3, 0x52fb58f4, 0x43fc48fd, 0x59f048f8, 0x3bff45f7, 0x39044205,
    0x47fe47fc, 0x4aff3a02, 0x45ff2cfc, 0x33f93e00, 0x2afa2ffc, 0x35fa29fd,
    0x4ef74c08, 0x340953f5, 0x5afb4300, 0x48f14301, 0x50f84bfb, 0x40eb53eb,
    0x40e71ff3, 0x4b095ee3, 0x4af83f11, 0x1bfe23fb, 0x41035b0d, 0x4d0845f9,
    0x3e0342f6, 0x51ec44fd, 0x07011e00, 0x4aeb17fd, 0x7ce94210, 0xee2c2511,
    0x7feade32, 0x2a002704, 0x1d0b2207, 0x25061f08, 0x28032a07, 0x2b0d2108,
    0x2f04240d, 0x3a023703, 0x2c083c06, 0x2a0e2c0b, 0x38043007, 0x250d3404,
    0x3a133109, 0x2d0c300a, 0x21144500, 0xee233f08, 0xfd1ce721, 0x001b0a18,
    0xd434f222, 0x1113e827, 0x1d24191f, 0x0f222118, 0x4916141e, 0x1f132214,
    0x10132c1b, 0x240f240f, 0x15191c15, 0x0c1f141e, 0x2a18101b, 0x380e5d00,
    0x261a390f, 0x73e87fe8, 0x3ef752ea, 0x3b003500, 0x59f355f2, 0x5cf55ef3,
    0x64eb7fe3, 0x43f439f2, 0x4df647f5, 0x58f055eb, 0x62f168e9, 0x52f67fdb,
    0x3df830f8, 0x46f942f8, 0x4ff64bf2, 0x5cf453f7, 0x4ffc6cee, 0x4bf045ea,
    0x3a013afe, 0x53f74ef3, 0x63f351fc, 0x26fa51f3, 0x3afa3ef3, 0x49f03bfe,
    0x56f34cf6, 0x57f653f7, 0x7fea0000, 0x78e77fe7, 0x72ed7fe5, 0x76e775e9,
    0x71e875e6, 0x78e176e4, 0x5ef67cdb, 0x63f666f1, 0x7fce6af3, 0x39115cfb,
    0x5ef356fb, 0x4dfe5bf4, 0x49ff4700, 0x51f94004, 0x390f4005, 0x44004301,
    0x440143f6, 0x40024d00, 0x4efb4400, 0x3b053707, 0x360e4102, 0x3c052c0f,
    0x4cfe4602, 0x460c56ee, 0x46f44005, 0x3805370b, 0x41024500, 0x36054afa,
    0x4cfa3607, 0x4dfe52f5, 0x2a194dfe, 0xf710f311, 0xeb1bf411, 0xd829e225,
    0xd130d72a, 0xd82ee027, 0xd72ecd34, 0xed2bd934, 0xc93d0b20, 0xce3ed238,
    0xec2dbd51, 0x0f1cfe23, 0x01270122, 0x2614111e, 0x360f2d12, 0xf0244f00,
    0xef25f225, 0x0f220120, 0x19180f1d, 0x101f1622, 0x1c1f1223, 0x1c242921,
    0x3e152f1b, 0x1a131f12, 0x17181824, 0x1e18101b, 0x29161d1f, 0x3c102a16,
    0x3c0e340f, 0x7bf04e03, 0x38163515, 0x21153d19, 0x3d113213, 0x4af84efd,
    0x48f648f7, 0x47f44bee, 0x46fb3ff5, 0x48f24bef, 0x35f843f0, 0x34f73bf2,
    0xfe0944f5, 0xfc1ff61e, 0x0721ff21, 0x17250c1f, 0x4014261f, 0x25f947f7,
    0x31f52cf8, 0x3bf438f6, 0x43f73ff8, 0x4ff644fa, 0x4af84efd, 0x48f648f7,
    0x47f44bee, 0x46fb3ff5, 0x48f24bef, 0x35f843f0, 0x34f73bf2, 0xfe0944f5,
    0xfc1ff61e, 0x0721ff21, 0x17250c1f, 0x4014261f, 0x00000000, 0x00000000,
    0x3602f114, 0xf1144a03, 0x4a033602, 0x68e97fe4, 0x36ff35fa, 0x00003307,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x3f002900, 0x3f003f00, 0x560453f7, 0x48f96100, 0x3e03290d, 0x37010b00,
    0x7fef4500, 0x520066f3, 0x6beb4af9, 0x7fe17fe5, 0x5fee7fe8, 0x72eb7fe5,
    0x7bef7fe2, 0x7af073f4, 0x3ff473f5, 0x54f144fe, 0x46fd68f3, 0x5af65df8,
    0x4aff7fe2, 0x5bf961fa, 0x38fc7fec, 0x4cf952fb, 0x5df97dea, 0x4dfd57f5,
    0x3ffc47fb, 0x54f444fc, 0x41f93ef9, 0x38053d08, 0x400142fe, 0x4efe3d00,
    0x34073201, 0x2c00230a, 0x2d01260b, 0x2c052e00, 0x3301111f, 0x131c3207,
    0x3e0e2110, 0x64f16cf3, 0x5bf365f3, 0x58f65ef4, 0x56f654f0, 0x57f353f9,
    0x46015eed, 0x4afb4800, 0x66f83b12, 0x5f0064f1, 0x48024bfc, 0x47fd4bf5,
    0x45f32e0f, 0x41003e00, 0x48f12515, 0x36103909, 0x480c3e00, 0x090f0018,
    0x120d1908, 0x130d090f, 0x120c250a, 0x21141d06, 0x2d041e0f, 0x3e003a01,
    0x260c3d07, 0x270f2d0b, 0x2c0d2a0b, 0x290c2d10, 0x221e310a, 0x370a2a12,
    0x2e113311, 0xed1a5900, 0xef1aef16, 0xec1ce71e, 0xe525e921, 0xe428e921,
    0xf521ef26, 0xfa29f128, 0x11290126, 0x031bfa1e, 0xf025161a, 0xf826fc23,
    0x0325fd26, 0x002a0526, 0x16271023, 0x251b300e, 0x440c3c15, 0x47fd6102,
    0x32fb2afa, 0x3efe36fd, 0x3f013a00, 0x4aff48fe, 0x43fb5bf7, 0x27fd1bfb,
    0x2e002cfe, 0x44f840f0, 0x4dfa4ef6, 0x5cf456f6, 0x3cf637f1, 0x41fc3efa,
    0x4cf849f4, 0x58f750f9, 0x61f56eef, 0x4ff554ec, 0x4afc49fa, 0x60f356f3,
    0x75ed61f5, 0x21fb4ef8, 0x35fe30fc, 0x47f33efd, 0x56f44ff6, 0x61f25af3,
    0x5dfa0000, 0x4ff854fa, 0x47ff4200, 0x3cfe3e00, 0x4bfb3bfe, 0x3afc3efd,
    0x4fff42f7, 0x44034700, 0x3ef92c0a, 0x280e240f, 0x1d0c1b10, 0x24142c01,
    0x2a052012, 0x3e0a3001, 0x40092e11, 0x61f568f4, 0x58f960f0, 0x55f955f8,
    0x58f355f7, 0x4dfd4204, 0x4cfa4cfd, 0x4cff3a0a, 0x63f953ff, 0x5f025ff2,
    0x4afb4c00, 0x4bf54600, 0x41004401, 0x3e0349f2, 0x44ff3e04, 0x370b4bf3,
    0x460c4005, 0x1306060f, 0x0e0c1007, 0x0b0d0d12, 0x100f0f0d, 0x170d170c,
    0x1a0e140f, 0x28112c0e, 0x11182f11, 0x16191515, 0x1d161b1f, 0x320e2313,
    0x3f07390a, 0x52fc4dfe, 0x45095efd, 0xdd246df4, 0xe620de24, 0xe02ce225,
    0xf122ee22, 0xf921f128, 0x0021fb23, 0x0d210226, 0x3a0d2317, 0x001afd1d,
    0xf91f1e16, 0xfd22f123, 0xff240322, 0x0b200522, 0x0c220523, 0x1d1e0b27,
    0x271d1a22, 0x151f4213, 0x32191f1f, 0x70ec78ef, 0x55f572ee, 0x59f25cf1,
    0x51f147e6, 0x440050f2, 0x38e846f2, 0x32e844e9, 0xf3174af5, 0xf128f31a,
    0x032cf231, 0x222c062d, 0x52133621, 0x17ff4bfd, 0x2b012201, 0x37fe3600,
    0x40013d00, 0x5cf74400, 0x61f36af2, 0x5af45af1, 0x49f658ee, 0x56f24ff7,
    0x46f649f6, 0x42fb45f6, 0x3afb40f7, 0xf6153b02, 0xf81cf518, 0x031dff1c,
    0x1423091d, 0x430e241d,
};

typedef struct RKVDEC341H264Context {
    RK_S32              dev_fd;
    RKVDEC341RegH264    reg;
    os_allocator        dma_allocator;
    void*               dma_allocator_ctx;
    AVFrame             *syntax_data;
    AVFrame             *cabac_data;
    AVFrame             *pps_data;
    AVFrame             *rps_data;
    AVFrame             *scaling_list_data;
    AVFrame             *errorinfo_data;
    AVFrame             *stream_data;
} RKVDEC341H264Context;

extern struct RKVDECDevice rkvdec341_h264;

static RK_S32 rkvdec341_h264_prepare(RKVDEC341H264Context *ctx, AVPacket* data, RKVDECPicParamsH264 *pp) {
    PutBitContext64 bp;    
    RK_U8 *ptr, *tmp_data;
    RK_S32 i, j;

    //stream
    if (ff_rkvdec_get_dma_size(ctx->stream_data) < data->size) {
        ctx->dma_allocator.free(ctx->dma_allocator_ctx, ctx->stream_data);
        ctx->stream_data->linesize[0] = data->size + RKVDEC341H264_DATA_SIZE; 
        ctx->dma_allocator.alloc(ctx->dma_allocator_ctx, ctx->stream_data);
    }
    ptr = ff_rkvdec_get_dma_ptr(ctx->stream_data);
    memcpy(ptr, data->data, data->size);
    ctx->stream_data->pkt_size = data->size;

    //sps
    ptr = ff_rkvdec_get_dma_ptr(ctx->pps_data);
    tmp_data = av_mallocz(32 + 10);
    init_put_bits_a64(&bp, tmp_data, 32);
    
    put_bits_a64(&bp, 4, -1);
    put_bits_a64(&bp, 8, -1);
    put_bits_a64(&bp, 1, -1);
    put_bits_a64(&bp, 2, pp->chroma_format_idc);
    put_bits_a64(&bp, 3, pp->bit_depth_luma_minus8 + 8);
    put_bits_a64(&bp, 3, pp->bit_depth_chroma_minus8 + 8);
    put_bits_a64(&bp, 1, 0);
    put_bits_a64(&bp, 4, pp->log2_max_frame_num_minus4);
    put_bits_a64(&bp, 5, pp->num_ref_frames);
    put_bits_a64(&bp, 2, pp->pic_order_cnt_type);
    put_bits_a64(&bp, 4, pp->log2_max_pic_order_cnt_lsb_minus4);
    put_bits_a64(&bp, 1, pp->delta_pic_order_always_zero_flag);
    put_bits_a64(&bp, 9, pp->frame_width_in_mbs_minus1 + 1);
    put_bits_a64(&bp, 9, pp->frame_height_in_mbs_minus1 + 1);
    put_bits_a64(&bp, 1, pp->frame_mbs_only_flag);
    put_bits_a64(&bp, 1, pp->mbaff_frame_flag);
    put_bits_a64(&bp, 1, pp->direct_8x8_inference_flag);

    put_bits_a64(&bp, 1, 0); // mvc_extension_enable
    put_bits_a64(&bp, 2, (pp->num_views_minus1 + 1));
    put_bits_a64(&bp, 10, pp->view_id[0]);
    put_bits_a64(&bp, 10, pp->view_id[1]);
    put_bits_a64(&bp, 1, pp->num_anchor_refs_l0[0]);
    if (pp->num_anchor_refs_l1[0]) {
        put_bits_a64(&bp, 10, pp->anchor_ref_l1[0][0]);
    } else {
        put_bits_a64(&bp, 10, 0); //!< anchor_ref_l1
    }
    put_bits_a64(&bp, 1, pp->num_non_anchor_refs_l0[0]);
    if (pp->num_non_anchor_refs_l0[0]) {
        put_bits_a64(&bp, 10, pp->non_anchor_ref_l0[0][0]);
    } else {
        put_bits_a64(&bp, 10, 0); //!< non_anchor_ref_l0
    }
    put_bits_a64(&bp, 1, pp->num_non_anchor_refs_l1[0]);
    if (pp->num_non_anchor_refs_l1[0]) {
        put_bits_a64(&bp, 10, pp->non_anchor_ref_l1[0][0]);
    } else {
        put_bits_a64(&bp, 10, 0);//!< non_anchor_ref_l1
    }
    put_align_a64(&bp, 32, 0);

    // pps
    put_bits_a64(&bp, 8, -1); //!< pps_pic_parameter_set_id
    put_bits_a64(&bp, 5, -1); //!< pps_seq_parameter_set_id
    put_bits_a64(&bp, 1, pp->entropy_coding_mode_flag);
    put_bits_a64(&bp, 1, pp->pic_order_present_flag);
    put_bits_a64(&bp, 5, pp->num_ref_idx_l0_active_minus1);
    put_bits_a64(&bp, 5, pp->num_ref_idx_l1_active_minus1);
    put_bits_a64(&bp, 1, pp->weighted_pred_flag);
    put_bits_a64(&bp, 2, pp->weighted_bipred_idc);
    put_bits_a64(&bp, 7, pp->pic_init_qp_minus26);
    put_bits_a64(&bp, 6, pp->pic_init_qs_minus26);
    put_bits_a64(&bp, 5, pp->chroma_qp_index_offset);
    put_bits_a64(&bp, 1, pp->deblocking_filter_control_present_flag);
    put_bits_a64(&bp, 1, pp->constrained_intra_pred_flag);
    put_bits_a64(&bp, 1, pp->redundant_pic_cnt_present_flag);
    put_bits_a64(&bp, 1, pp->transform_8x8_mode_flag);
    put_bits_a64(&bp, 5, pp->second_chroma_qp_index_offset);
    put_bits_a64(&bp, 1, pp->scaleing_list_enable_flag);
    put_bits_a64(&bp, 32, ff_rkvdec_get_dma_fd(ctx->syntax_data) | 
        ((ff_rkvdec_get_dma_ptr(ctx->scaling_list_data) - ff_rkvdec_get_dma_ptr(ctx->syntax_data))<< 10));

    for (i = 0; i < 16; i++) {
        int is_long_term = pp->DPB[i].valid ? pp->DPB[i].lt : 0;
        put_bits_a64(&bp, 1, is_long_term);
    }
    for (i = 0; i < 16; i++) {
        int voidx = pp->DPB[i].valid ? pp->ref_pic_layerid_list[i] : 0;
        put_bits_a64(&bp, 1, voidx);
    }
    put_align_a64(&bp, 64, 0);

    for(i = 0; i < 256; i++) {
        memcpy(ptr + i * 32, tmp_data, 32);
    }
    av_free(tmp_data);

    //rps
    ptr = ff_rkvdec_get_dma_ptr(ctx->rps_data);
    init_put_bits_a64(&bp, ptr, RKVDEC341H264_RPS_SIZE);
    
    for (i = 0; i < 16; i++) {
        RK_U32 frame_num_wrap = 0;
        if (pp->DPB[i].valid) {
            RK_U32 max_frame_num = 1 << (pp->log2_max_frame_num_minus4 + 4);
            if (pp->DPB[i].lt)
                frame_num_wrap = pp->DPB[i].frame_idx;
            else
                frame_num_wrap = pp->DPB[i].frame_idx > pp->curr_pic.frame_idx ? 
                                (pp->DPB[i].frame_idx - max_frame_num) : pp->DPB[i].frame_idx;
        }
        put_bits_a64(&bp, 16, frame_num_wrap);
    }

    for (i = 0; i < 16; i++) {
        put_bits_a64(&bp, 1, 0);
    }

    for (i = 0; i < 16; i++) {
        put_bits_a64(&bp, 1, pp->ref_pic_layerid_list[i]);
    }

    for (i = 0; i < 32; i++) {
        RK_U32 dpb_idx = 0, dpb_valid = 0, bottom_flag = 0, voidx = 0;
        RKVDECPicture *pic = &pp->ref_frame_list[0][i];
        
        if (pic->valid && get_rkvdec_picture_index2(pp->DPB, pic) >= 0) {
            dpb_valid = 1;
            dpb_idx = get_rkvdec_picture_index2(pp->DPB, pic);
            bottom_flag = (pic->pic_struct == 2);
            voidx = pp->ref_pic_layerid_list[dpb_idx];
        }

        put_bits_a64(&bp, 5, dpb_idx | (dpb_valid << 4));
        put_bits_a64(&bp, 1, bottom_flag);
        put_bits_a64(&bp, 1, voidx);
    }

    for (j = 0; j < 2; j++) {
        for (i = 0; i < 32; i++) {
            RK_U32 dpb_idx = 0, dpb_valid = 0, bottom_flag = 0, voidx = 0;
            RKVDECPicture *pic = &pp->ref_frame_list[j][i];

            if (pic->valid && get_rkvdec_picture_index2(pp->DPB, pic) >= 0) {
                dpb_valid = 1;
                dpb_idx = get_rkvdec_picture_index2(pp->DPB, pic);
                bottom_flag = (pic->pic_struct == 2);
                voidx = pp->ref_pic_layerid_list[dpb_idx];
            }
            
            put_bits_a64(&bp, 5, dpb_idx | (dpb_valid << 4));
            put_bits_a64(&bp, 1, bottom_flag);
            put_bits_a64(&bp, 1, voidx);

        }
    }
    put_align_a64(&bp, 128, 0);

    // scaling list
    if (pp->scaleing_list_enable_flag) {
        ptr = ff_rkvdec_get_dma_ptr(ctx->scaling_list_data);

        init_put_bits_a64(&bp, ptr, RKVDEC341H264_SCALING_LIST_SIZE);
        memset(ptr, 0, RKVDEC341H264_SCALING_LIST_SIZE);
        for (i = 0; i < 6; i++) {
            for (j = 0; j < 16; j++)
                put_bits_a64(&bp, 8, pp->scaling_lists4x4[i][j]);
        }

        for (j = 0; j < 64; j++)
            put_bits_a64(&bp, 8, pp->scaling_lists8x8[0][j]);

        for (j = 0; j < 64; j++)
            put_bits_a64(&bp, 8, pp->scaling_lists8x8[3][j]);
    }

    // regs
    memset(&ctx->reg, 0, sizeof(RKVDEC341RegH264));

    ctx->reg.swreg2_sysctrl.sw_dec_mode = 1;
    ctx->reg.swreg3_picpar.sw_slice_num_lowbits = 0x7ff;
    ctx->reg.swreg3_picpar.sw_slice_num_highbit = 1;    
    ctx->reg.swreg5_stream_rlc_len.sw_stream_len = ALIGN(ctx->stream_data->pkt_size, 16);
    ctx->reg.swreg3_picpar.sw_y_hor_virstride = ALIGN(pp->frame_width, 16) / 16;
    ctx->reg.swreg3_picpar.sw_uv_hor_virstride =  ALIGN(pp->frame_width, 16) / 16;
    ctx->reg.swreg8_y_virstride.sw_y_virstride = ALIGN(ALIGN(pp->frame_width, 16) * ALIGN(pp->frame_height, 16), 16) / 16;
    ctx->reg.swreg9_yuv_virstride.sw_yuv_virstride = ALIGN(ALIGN(pp->frame_width, 16) * ALIGN(pp->frame_height, 16) * 3 / 2, 16) / 16;
    ctx->reg.swreg40_cur_poc.sw_cur_poc = pp->curr_pic.field_poc[0];
    ctx->reg.swreg74_h264_cur_poc1.sw_h264_cur_poc1 = pp->curr_pic.field_poc[1];
    ctx->reg.swreg7_decout_base.sw_decout_base = pp->curr_pic.index;
    ctx->reg.swreg78_colmv_cur_base.sw_colmv_base = pp->curr_mv;

    for (i = 0; i < 15; i++) {
        ctx->reg.swreg25_39_refer0_14_poc[i] = (i & 1) ? pp->DPB[i / 2].field_poc[1] : pp->DPB[i / 2].field_poc[0];
        ctx->reg.swreg49_63_refer15_29_poc[i] = (i & 1) ? pp->DPB[(i + 15) / 2].field_poc[0] : pp->DPB[(i + 15) / 2].field_poc[1];
        ctx->reg.swreg10_24_refer0_14_base[i].sw_ref_field = pp->DPB[i].field_picture;
        ctx->reg.swreg10_24_refer0_14_base[i].sw_ref_topfield_used = !!(pp->DPB[i].reference & 1);
        ctx->reg.swreg10_24_refer0_14_base[i].sw_ref_botfield_used = !!(pp->DPB[i].reference & 2);
        ctx->reg.swreg25_39_refer0_14_poc[i] = ctx->reg.swreg25_39_refer0_14_poc[i];
        ctx->reg.swreg10_24_refer0_14_base[i].sw_ref_colmv_use_flag = 0x01;

        if (pp->DPB[i].valid)
            ctx->reg.swreg10_24_refer0_14_base[i].sw_refer_base = pp->DPB[i].index;
        else
            ctx->reg.swreg10_24_refer0_14_base[i].sw_refer_base = 
                i ? ctx->reg.swreg10_24_refer0_14_base[i - 1].sw_refer_base : pp->curr_pic.index;
    }
    ctx->reg.swreg72_refer30_poc = pp->DPB[15].field_poc[0];
    ctx->reg.swreg73_refer31_poc = pp->DPB[15].field_poc[1];
    ctx->reg.swreg48_refer15_base.sw_ref_field = pp->DPB[15].field_picture;
    ctx->reg.swreg48_refer15_base.sw_ref_topfield_used = !!(pp->DPB[i].reference & 1);
    ctx->reg.swreg48_refer15_base.sw_ref_botfield_used = !!(pp->DPB[i].reference & 2);
    ctx->reg.swreg48_refer15_base.sw_ref_colmv_use_flag = 0x01;
    if (pp->DPB[15].valid)
        ctx->reg.swreg48_refer15_base.sw_refer_base = pp->DPB[15].index;
    else
        ctx->reg.swreg48_refer15_base.sw_refer_base = ctx->reg.swreg10_24_refer0_14_base[14].sw_refer_base;

    ctx->reg.swreg4_strm_rlc_base.sw_strm_rlc_base = ff_rkvdec_get_dma_fd(ctx->stream_data);
    ctx->reg.swreg6_cabactbl_prob_base.sw_cabactbl_base = ff_rkvdec_get_dma_fd(ctx->syntax_data);
    ctx->reg.swreg41_rlcwrite_base.sw_rlcwrite_base = ctx->reg.swreg4_strm_rlc_base.sw_strm_rlc_base;

    ctx->reg.swreg42_pps_base.sw_pps_base = ff_rkvdec_get_dma_fd(ctx->syntax_data) + 
        ((ff_rkvdec_get_dma_ptr(ctx->pps_data) - ff_rkvdec_get_dma_ptr(ctx->syntax_data)) << 10);
    ctx->reg.swreg43_rps_base.sw_rps_base = ff_rkvdec_get_dma_fd(ctx->syntax_data) + 
        ((ff_rkvdec_get_dma_ptr(ctx->rps_data) - ff_rkvdec_get_dma_ptr(ctx->syntax_data)) << 10);
    ctx->reg.swreg75_h264_errorinfo_base.sw_errorinfo_base = ff_rkvdec_get_dma_fd(ctx->syntax_data) + 
        ((ff_rkvdec_get_dma_ptr(ctx->errorinfo_data) - ff_rkvdec_get_dma_ptr(ctx->syntax_data)) << 10);

    for (i = 0; i < 16; i++) {
        if (pp->ref_colmv_list[i] > 0)
            ctx->reg.swreg79_94_colmv0_15_base[i].sw_colmv_base = pp->ref_colmv_list[i];
        else
            ctx->reg.swreg79_94_colmv0_15_base[i].sw_colmv_base = i ? 
            ctx->reg.swreg79_94_colmv0_15_base[i - 1].sw_colmv_base : pp->curr_mv;
    }

    if (ctx->reg.swreg78_colmv_cur_base.sw_colmv_base)
        ctx->reg.swreg2_sysctrl.sw_colmv_mode = 1;

    ctx->reg.swreg67_fpgadebug_reset.sw_resetn = 0xff;
    ctx->reg.swreg44_strmd_error_en.sw_strmd_error_e = 0xfffffff;
    ctx->reg.swreg77_h264_error_e.sw_h264_error_en_highbits = 0x3fffffff;
    ctx->reg.swreg1_int.sw_dec_e = 1;
    ctx->reg.swreg1_int.sw_dec_timeout_e = 1;
    ctx->reg.swreg1_int.sw_buf_empty_en = 1;

    return 0;
}

static RK_S32 rkvdec341_h264_perform(void *p) {    
    RKVDEC341H264Context *ctx = (RKVDEC341H264Context*)p;
    RKVDECHwReq req;

    req.req = (RK_U32*)&ctx->reg;
    req.size = 95 * sizeof(RK_U32);

    if (ioctl(ctx->dev_fd, RKVDEC_IOC_SET_REG, &req))
        return AVERROR_INVALIDDATA;
    if (ioctl(ctx->dev_fd, RKVDEC_IOC_GET_REG, &req))
        return AVERROR_INVALIDDATA;

    if (ctx->reg.swreg1_int.sw_dec_error_sta
        || (!ctx->reg.swreg1_int.sw_dec_rdy_sta)
        || ctx->reg.swreg1_int.sw_dec_empty_sta
        || ctx->reg.swreg45_strmd_error_status.sw_strmd_error_status
        || ctx->reg.swreg45_strmd_error_status.sw_colmv_error_ref_picidx
        || ctx->reg.swreg76_h264_errorinfo_num.sw_strmd_detect_error_flag)
        return AVERROR_INVALIDDATA;

    return 0;
}

static RK_S32 rkvdec341_h264_uninit(void* p) {
    RKVDEC341H264Context *ctx = (RKVDEC341H264Context*)p;
    if (ctx->dev_fd) {
        close(ctx->dev_fd);
        ctx->dev_fd = -1;
    }
    ctx->dma_allocator.free(ctx->dma_allocator_ctx, ctx->syntax_data);
    ctx->dma_allocator.free(ctx->dma_allocator_ctx, ctx->stream_data);
    ctx->dma_allocator.close(ctx->dma_allocator_ctx);
    av_freep(&ctx->syntax_data);
    av_freep(&ctx->cabac_data);
    av_freep(&ctx->pps_data);
    av_freep(&ctx->rps_data);
    av_freep(&ctx->scaling_list_data);
    av_freep(&ctx->errorinfo_data);
    av_freep(&ctx->stream_data);

    return 0;
}

static RK_S32 rkvdec341_h264_init(void *p) {
    RKVDEC341H264Context *ctx = (RKVDEC341H264Context*)p;
    ctx->dev_fd = open(rkvdec341_h264.dev, O_RDWR);
    if (ctx->dev_fd <= 0)
        return AVERROR_DECODER_NOT_FOUND;

    if(ioctl(ctx->dev_fd, RKVDEC_IOC_SET_CLIENT_TYPE, 0x1)) {
        if (ioctl(ctx->dev_fd, RKVDEC_IOC_SET_CLIENT_TYPE_U32, 0x1)) {
            return AVERROR_DECODER_NOT_FOUND;
        }
    }

    ctx->dma_allocator = allocator_drm;
    if (ctx->dma_allocator.open(&ctx->dma_allocator_ctx, 1))
        return AVERROR_UNKNOWN;

    ctx->syntax_data = av_frame_alloc();
    ctx->syntax_data->linesize[0] = RKVDEC341H264_CABAC_TAB_SIZE + 
                                    RKVDEC341H264_SPSPPS_SIZE + 
                                    RKVDEC341H264_RPS_SIZE + 
                                    RKVDEC341H264_SCALING_LIST_SIZE + 
                                    RKVDEC341H264_ERROR_INFO_SIZE;
    ctx->dma_allocator.alloc(ctx->dma_allocator_ctx, ctx->syntax_data);

    ctx->cabac_data = av_frame_alloc();
    ctx->cabac_data->linesize[0] = RKVDEC341H264_CABAC_TAB_SIZE;
    ctx->cabac_data->data[0] = ff_rkvdec_get_dma_ptr(ctx->syntax_data);
    memcpy(ctx->cabac_data->data[0], RKVDEC341_cabac_table, sizeof(RKVDEC341_cabac_table));

    ctx->pps_data = av_frame_alloc();
    ctx->pps_data->linesize[0] = RKVDEC341H264_SPSPPS_SIZE;
    ctx->pps_data->data[0] = ff_rkvdec_get_dma_ptr(ctx->syntax_data) + RKVDEC341H264_CABAC_TAB_SIZE;

    ctx->rps_data = av_frame_alloc();
    ctx->rps_data->linesize[0] = RKVDEC341H264_RPS_SIZE;
    ctx->rps_data->data[0] = ff_rkvdec_get_dma_ptr(ctx->pps_data) + RKVDEC341H264_SPSPPS_SIZE;

    ctx->scaling_list_data = av_frame_alloc();
    ctx->scaling_list_data->linesize[0] = RKVDEC341H264_SCALING_LIST_SIZE;
    ctx->scaling_list_data->data[0] = ff_rkvdec_get_dma_ptr(ctx->rps_data) + RKVDEC341H264_RPS_SIZE;

    ctx->errorinfo_data = av_frame_alloc();
    ctx->errorinfo_data->linesize[0] = RKVDEC341H264_ERROR_INFO_SIZE;
    ctx->errorinfo_data->data[0] = ff_rkvdec_get_dma_ptr(ctx->scaling_list_data) + RKVDEC341H264_SCALING_LIST_SIZE;

    ctx->stream_data = av_frame_alloc();
    ctx->stream_data->linesize[0] = RKVDEC341H264_DATA_SIZE;
    ctx->dma_allocator.alloc(ctx->dma_allocator_ctx, ctx->stream_data);

    return 0;
}

struct RKVDECDevice rkvdec341_h264 = {
    .name   = "vdpu341",
    .dev    = "/dev/rkvdec",
    .priv_data_size = sizeof(RKVDEC341H264Context),
    .init       = rkvdec341_h264_init,
    .uninit     = rkvdec341_h264_uninit,
    .prepare    = rkvdec341_h264_prepare,
    .perform    = rkvdec341_h264_perform,
};

