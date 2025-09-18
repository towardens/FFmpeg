/*
 * NTSC CRT simulation video filter - Multi-format (RGB/YUV) version
 * All credit goes to https://github.com/LMP88959/NTSC-CRT
 * This is just a ffmpeg filter using the hardwork of the original author.
 */

#include "avfilter.h"
#include "libavutil/internal.h"
#include "video.h"
#include "formats.h"
#include "libavutil/opt.h"
#include "libavutil/pixdesc.h"
#include "libavutil/mem.h"
#include "libavutil/common.h"
#include "crt/crt_core.h"

typedef struct CRTContext {
    const AVClass *class;

    // CRT library structures
    struct CRT crt;
    struct NTSC_SETTINGS ntsc;

    // Filter options
    int color;
    int noise;
    int hue;
    int blend;
    int scanlines;
    int raw;

    // Internal state
    int field;
    uint8_t *output_buffer;
    int buffer_size;
    int initialized;
} CRTContext;

#define OFFSET(x) offsetof(CRTContext, x)
#define FLAGS AV_OPT_FLAG_VIDEO_PARAM|AV_OPT_FLAG_FILTERING_PARAM

static const AVOption crt_options[] = {
    {"color", "Enable color mode", OFFSET(color), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS},
    {"noise", "Noise level", OFFSET(noise), AV_OPT_TYPE_INT, {.i64 = 12}, 0, 100, FLAGS},
    {"hue", "Hue adjustment", OFFSET(hue), AV_OPT_TYPE_INT, {.i64 = 0}, -180, 180, FLAGS},
    {"blend", "Enable frame blending", OFFSET(blend), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS},
    {"scanlines", "Enable scanlines", OFFSET(scanlines), AV_OPT_TYPE_BOOL, {.i64 = 1}, 0, 1, FLAGS},
    {"raw", "Raw NTSC output mode", OFFSET(raw), AV_OPT_TYPE_BOOL, {.i64 = 0}, 0, 1, FLAGS},
    {NULL}
};

AVFILTER_DEFINE_CLASS(crt);

// Supported input/output formats
static int query_formats(const AVFilterContext *ctx,
                         AVFilterFormatsConfig **cfg_in,
                         AVFilterFormatsConfig **cfg_out) {
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_BGRA,
        AV_PIX_FMT_RGBA,
        AV_PIX_FMT_BGR24,
        AV_PIX_FMT_RGB24,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_YUVJ420P,
        AV_PIX_FMT_NONE
    };

    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);

    return ff_set_common_formats2(ctx, cfg_in, cfg_out, fmts_list);
}


// Convert YUV420P to RGB24 (helper)
static void yuv_to_rgb24(const AVFrame *in, uint8_t *rgb) {
    int w = in->width, h = in->height;
    for (int y = 0; y < h; y++) {
        const uint8_t *py = in->data[0] + y * in->linesize[0];
        const uint8_t *pu = in->data[1] + (y >> 1) * in->linesize[1];
        const uint8_t *pv = in->data[2] + (y >> 1) * in->linesize[2];
        uint8_t *dst = rgb + y * w * 3;
        for (int x = 0; x < w; x++) {
            int Y = py[x];
            int U = pu[x >> 1] - 128;
            int V = pv[x >> 1] - 128;
            int R = av_clip_uint8(Y + 1.402 * V);
            int G = av_clip_uint8(Y - 0.344136 * U - 0.714136 * V);
            int B = av_clip_uint8(Y + 1.772 * U);
            dst[3 * x + 0] = R;
            dst[3 * x + 1] = G;
            dst[3 * x + 2] = B;
        }
    }
}

// Convert RGB24 to BGRA
static void rgb24_to_bgra(uint8_t *rgb, uint8_t *bgra, int w, int h) {
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int idx_rgb = (y * w + x) * 3;
            int idx_bgra = (y * w + x) * 4;
            bgra[idx_bgra + 0] = rgb[idx_rgb + 2]; // B
            bgra[idx_bgra + 1] = rgb[idx_rgb + 1]; // G
            bgra[idx_bgra + 2] = rgb[idx_rgb + 0]; // R
            bgra[idx_bgra + 3] = 255; // A
        }
    }
}

// Convert BGRA to YUV420P
static void bgra_to_yuv420p(uint8_t *bgra, AVFrame *out) {
    int w = out->width, h = out->height;
    for (int y = 0; y < h; y++) {
        uint8_t *Yp = out->data[0] + y * out->linesize[0];
        uint8_t *Up = out->data[1] + (y >> 1) * out->linesize[1];
        uint8_t *Vp = out->data[2] + (y >> 1) * out->linesize[2];
        for (int x = 0; x < w; x++) {
            int idx = (y * w + x) * 4;
            int R = bgra[idx + 2], G = bgra[idx + 1], B = bgra[idx + 0];
            int Y = 0.299 * R + 0.587 * G + 0.114 * B;
            int U = -0.169 * R - 0.331 * G + 0.5 * B + 128;
            int V = 0.5 * R - 0.419 * G - 0.081 * B + 128;
            Yp[x] = av_clip_uint8(Y);
            if ((y & 1) == 0 && (x & 1) == 0) {
                Up[x >> 1] = av_clip_uint8(U);
                Vp[x >> 1] = av_clip_uint8(V);
            }
        }
    }
}

static int config_props(AVFilterLink *inlink) {
    AVFilterContext *ctx = inlink->dst;
    CRTContext *s = ctx->priv;

    if (s->initialized) {
        av_freep(&s->output_buffer);
        s->initialized = 0;
    }
    s->crt.blend = s->blend;
    s->crt.scanlines = s->scanlines;

    s->buffer_size = inlink->w * inlink->h * 4;
    s->output_buffer = av_malloc(s->buffer_size);
    if (!s->output_buffer) {
        av_log(ctx, AV_LOG_ERROR, "Could not allocate output buffer\n");
        return AVERROR(ENOMEM);
    }

    crt_init(&s->crt, inlink->w, inlink->h, CRT_PIX_FORMAT_BGRA, s->output_buffer);
    s->initialized = 1;

    av_log(ctx, AV_LOG_VERBOSE,
           "CRT filter initialized: %dx%d\n", inlink->w, inlink->h);

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in) {
    AVFilterContext *ctx = inlink->dst;
    AVFilterLink *outlink = ctx->outputs[0];
    CRTContext *s = ctx->priv;
    AVFrame *out;
    int ret;

    if (!s->initialized) {
        ret = AVERROR_BUG;
        goto fail;
    }

    out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
    if (!out) {
        ret = AVERROR(ENOMEM);
        goto fail;
    }
    av_frame_copy_props(out, in);


    if (in->format != AV_PIX_FMT_BGRA) {
        int rgb_size = in->width * in->height * 3;
        uint8_t *rgb = av_malloc(rgb_size);
        if (!rgb) {
            ret = AVERROR(ENOMEM);
            goto fail;
        }

        if (in->format == AV_PIX_FMT_RGBA) {
            for (int y = 0; y < in->height; y++)
                for (int x = 0; x < in->width; x++) {
                    uint8_t *src = in->data[0] + y * in->linesize[0] + x * 4;
                    uint8_t *dst = rgb + (y * in->width + x) * 3;
                    dst[0] = src[0];
                    dst[1] = src[1];
                    dst[2] = src[2];
                }
        } else if (in->format == AV_PIX_FMT_RGB24 || in->format == AV_PIX_FMT_BGR24) {
            for (int y = 0; y < in->height; y++)
                memcpy(rgb + y * in->width * 3, in->data[0] + y * in->linesize[0], in->width * 3);
        } else if (in->format == AV_PIX_FMT_YUV420P || in->format == AV_PIX_FMT_YUVJ420P) {
            yuv_to_rgb24(in, rgb);
        } else {
            av_log(ctx, AV_LOG_ERROR, "Unsupported input format\n");
            ret = AVERROR(EINVAL);
            goto fail;
        }
        rgb24_to_bgra(rgb, s->output_buffer, in->width, in->height);
        av_freep(&rgb);
    }

    // Set up NTSC settings for this frame
    s->ntsc.data = s->output_buffer;
    s->ntsc.format = CRT_PIX_FORMAT_BGRA;
    s->ntsc.w = in->width;
    s->ntsc.h = in->height;
    s->ntsc.as_color = s->color;
    s->ntsc.field = s->field & 1;
    s->crt.scanlines = s->scanlines;
    s->ntsc.raw = s->raw;
    s->ntsc.hue = s->hue;

    // Toggle frame counter for even fields
    if (s->ntsc.field == 0) {
        s->ntsc.frame ^= 1;
    }


    crt_modulate(&s->crt, &s->ntsc);
    crt_demodulate(&s->crt, s->noise);

    s->field ^= 1;

    if (out->format == AV_PIX_FMT_BGRA) {
        for (int y = 0; y < out->height; y++)
            memcpy(out->data[0] + y * out->linesize[0], s->output_buffer + y * out->width * 4, out->width * 4);
    } else if (out->format == AV_PIX_FMT_YUV420P || out->format == AV_PIX_FMT_YUVJ420P) {
        bgra_to_yuv420p(s->output_buffer, out);
    } else {
        av_log(ctx, AV_LOG_ERROR, "Unsupported output format\n");
        ret = AVERROR(EINVAL);
        goto fail;
    }


    av_frame_free(&in);
    return ff_filter_frame(outlink, out);

fail:
    av_frame_free(&in);
    av_frame_free(&out);
    return ret;
}

static av_cold void uninit(AVFilterContext *ctx) {
    CRTContext *s = ctx->priv;
    if (s->output_buffer)
        av_freep(&s->output_buffer);
    s->initialized = 0;
}

static const AVFilterPad crt_inputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
        .filter_frame = filter_frame,
        .config_props = config_props,
    },
};

static const AVFilterPad crt_outputs[] = {
    {
        .name = "default",
        .type = AVMEDIA_TYPE_VIDEO,
    },
};

const FFFilter ff_vf_crt = {
    .p.name = "crt",
    .p.description = NULL_IF_CONFIG_SMALL("Apply NTSC CRT simulation effect."),
    .p.priv_class = &crt_class,
    .priv_size = sizeof(CRTContext),
    .uninit = uninit,
    FILTER_INPUTS(crt_inputs),
    FILTER_OUTPUTS(crt_outputs),
    FILTER_QUERY_FUNC2(query_formats),
};
