/*
 * Copyright (C) 2005 Robert Edele <yartrebo@earthlink.net>
 * Copyright (C) 2012 Stefano Sabatini
 * Copyright (C) 2017 David Christenson
 *
 * This library is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2.1 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * Advanced blur-based logo removing filter
 *
 * This filter loads an image mask file showing where a logo is and
 * uses a blur transform to remove the logo.
 *
 * Based on the libmpcodecs remove-logo filter by Robert Edele.
 */

/**
 * This code implements a filter to remove annoying TV logos and other annoying
 * images placed onto a video stream. It works by filling in the pixels that
 * comprise the logo with neighboring pixels. The transform is very loosely
 * based on a gaussian blur, but it is different enough to merit its own
 * paragraph later on. It is a major improvement on the old delogo filter as it
 * both uses a better blurring algorithm and uses a bitmap to use an arbitrary
 * and generally much tighter fitting shape than a rectangle.
 *
 * The logo removal algorithm has two key points. The first is that it
 * distinguishes between pixels in the logo and those not in the logo by using
 * the passed-in bitmap. Pixels not in the logo are copied over directly without
 * being modified and they also serve as source pixels for the logo
 * fill-in. Pixels inside the logo have the mask applied.
 *
 * At init-time the bitmap is reprocessed internally, and the distance to the
 * nearest edge of the logo (Manhattan distance), along with a little extra to
 * remove rough edges, is stored in each pixel. This is done using an in-place
 * erosion algorithm, and incrementing each pixel that survives any given
 * erosion.  Once every pixel is eroded, the maximum value is recorded, and a
 * set of masks from size 0 to this size are generaged. The masks are circular
 * binary masks, where each pixel within a radius N (where N is the size of the
 * mask) is a 1, and all other pixels are a 0. Although a gaussian mask would be
 * more mathematically accurate, a binary mask works better in practice because
 * we generally do not use the central pixels in the mask (because they are in
 * the logo region), and thus a gaussian mask will cause too little blur and
 * thus a very unstable image.
 *
 * The mask is applied in a special way. Namely, only pixels in the mask that
 * line up to pixels outside the logo are used. The dynamic mask size means that
 * the mask is just big enough so that the edges touch pixels outside the logo,
 * so the blurring is kept to a minimum and at least the first boundary
 * condition is met (that the image function itself is continuous), even if the
 * second boundary condition (that the derivative of the image function is
 * continuous) is not met. A masking algorithm that does preserve the second
 * boundary coundition (perhaps something based on a highly-modified bi-cubic
 * algorithm) should offer even better results on paper, but the noise in a
 * typical TV signal should make anything based on derivatives hopelessly noisy.
 */

#include <stdint.h>  // for uint8_t
#include <stdlib.h>  // for NULL, free, malloc

#include <vapoursynth/VSHelper.h>     // for VS_ALIGNED_FREE, VS_ALIGNED_MALLOC
#include <vapoursynth/VapourSynth.h>  // for VSAPI, VSVideoInfo, VSFormat

#define ALIGN 32

typedef struct BoundingBox {
    int x1, x2, y1, y2;
} BoundingBox;

typedef struct RemoveLogoData {
    VSNodeRef *node;
    const VSVideoInfo *vi;
    const char *filename;

    /* Stores our collection of masks. The first is for an array of
       the second for the y axis, and the third for the x axis. */
    int ***mask;
    int max_mask_size;

    uint8_t *full_mask_data;
    BoundingBox full_mask_bbox;
    uint8_t *half_mask_data;
    BoundingBox half_mask_bbox;
} RemoveLogoData;

static int calculate_bounding_box(BoundingBox *bbox, const uint8_t *data, int w,
                                  int h) {
    int x, y;
    int start_x;
    int start_y;
    int end_x;
    int end_y;
    const uint8_t *line;

    // left bound
    for (start_x = 0; start_x < w; start_x++)
        for (y = 0; y < h; y++)
            if ((data[y * w + start_x] > 0)) goto outl;

outl:
    if (start_x == w)  // no points found
        return 0;

    // right bound
    for (end_x = w - 1; end_x >= start_x; end_x--)
        for (y = 0; y < h; y++)
            if ((data[y * w + end_x] > 0)) goto outr;

outr:
    // top bound
    line = data;
    for (start_y = 0; start_y < h; start_y++) {
        for (x = 0; x < w; x++)
            if (line[x] > 0) goto outt;
        line += w;
    }

outt:
    // bottom bound
    line = data + (h - 1) * w;
    for (end_y = h - 1; end_y >= start_y; end_y--) {
        for (x = 0; x < w; x++)
            if (line[x] > 0) goto outb;
        line -= w;
    }

outb:
    bbox->x1 = start_x;
    bbox->y1 = start_y;
    bbox->x2 = end_x;
    bbox->y2 = end_y;
    return 1;
}

static int load_mask(uint8_t **data, int *w, int *h, const char *filename,
                     VSMap *out, VSCore *core, const VSAPI *vsapi) {
    VSPlugin *imwri = vsapi->getPluginById("com.vapoursynth.imwri", core);
    if (imwri == NULL) {
        vsapi->setError(out, "RemoveLogo: imwri plugin not found");
        return -1;
    }

    VSMap *args = vsapi->createMap();
    vsapi->propSetData(args, "filename", filename, -1, paReplace);

    VSMap *image = vsapi->invoke(imwri, "Read", args);
    vsapi->freeMap(args);
    if (vsapi->getError(image)) {
        vsapi->setError(out, vsapi->getError(image));
        vsapi->freeMap(image);
        return -1;
    }

    VSNodeRef *node = vsapi->propGetNode(image, "clip", 0, NULL);
    vsapi->freeMap(image);

    const VSFormat *format = vsapi->getVideoInfo(node)->format;
    if (format->colorFamily == cmGray || format->colorFamily == cmYUV ||
        format->colorFamily == cmYCoCg) {
        if (format->sampleType == stFloat) {
            VSPlugin *resize =
                vsapi->getPluginById("com.vapoursynth.resize", core);
            vsapi->propSetNode(args, "clip", node, paReplace);
            vsapi->freeNode(node);
            vsapi->propSetInt(args, "format", pfGray8, paReplace);
            image = vsapi->invoke(resize, "Bicubic", args);
            vsapi->freeMap(args);
            if (vsapi->getError(image)) {
                vsapi->setError(out, vsapi->getError(image));
                vsapi->freeMap(image);
                vsapi->freeNode(node);
                return -1;
            }
            node = vsapi->propGetNode(image, "clip", 0, NULL);
            vsapi->freeMap(image);
        }
    } else if (format->colorFamily == cmRGB) {
        VSPlugin *resize = vsapi->getPluginById("com.vapoursynth.resize", core);
        vsapi->propSetNode(args, "clip", node, paReplace);
        vsapi->freeNode(node);
        vsapi->propSetInt(args, "format", pfYUV444P8, paReplace);
        vsapi->propSetInt(args, "matrix", 7, paReplace);
        image = vsapi->invoke(resize, "Bicubic", args);
        vsapi->freeMap(args);
        if (vsapi->getError(image)) {
            vsapi->setError(out, vsapi->getError(image));
            vsapi->freeMap(image);
            vsapi->freeNode(node);
            return -1;
        }
        node = vsapi->propGetNode(image, "clip", 0, NULL);
        vsapi->freeMap(image);
    } else {
        vsapi->freeMap(image);
        vsapi->freeNode(node);
        vsapi->setError(out, "RemoveLogo: mask's color family is unsupported");
        return -1;
    }
    const VSFrameRef *frame = vsapi->getFrame(0, node, NULL, 0);
    vsapi->freeNode(node);
    if (frame == NULL) {
        vsapi->setError(out, "RemoveLogo: could not retrieve mask frame");
        return -1;
    }

    *w = vsapi->getFrameWidth(frame, 0);
    *h = vsapi->getFrameHeight(frame, 0);
    VS_ALIGNED_MALLOC(data, *w * *h, ALIGN);
    int stride = vsapi->getStride(frame, 0);
    vs_bitblt(*data, stride, vsapi->getReadPtr(frame, 0), stride, *w, *h);
    vsapi->freeFrame(frame);
    return 0;
}

/**
 * Choose a slightly larger mask size to improve performance.
 *
 * This function maps the absolute minimum mask size needed to the
 * mask size we'll actually use. f(x) = x (the smallest that will
 * work) will produce the sharpest results, but will be quite
 * jittery. f(x) = 1.25x (what I'm using) is a good tradeoff in my
 * opinion. This will calculate only at init-time, so you can put a
 * long expression here without effecting performance.
 */
#define apply_mask_fudge_factor(x) (((x) >> 2) + (x))

/**
 * Pre-process an image to give distance information.
 *
 * This function takes a bitmap image and converts it in place into a
 * distance image. A distance image is zero for pixels outside of the
 * logo and is the Manhattan distance (|dx| + |dy|) from the logo edge
 * for pixels inside of the logo. This will overestimate the distance,
 * but that is safe, and is far easier to implement than a proper
 * pythagorean distance since I'm using a modified erosion algorithm
 * to compute the distances.
 */
static void convert_mask_to_strength_mask(uint8_t *data, int linesize, int w,
                                          int h, int min_val,
                                          int *max_mask_size) {
    int x, y;

    /* How many times we've gone through the loop. Used in the
       in-place erosion algorithm and to get us max_mask_size later on. */
    int current_pass = 0;

    // set all non-zero values to 1
    for (y = 0; y < h; y++)
        for (x = 0; x < w; x++)
            data[y * linesize + x] = data[y * linesize + x] > min_val;

    /* For each pass, if a pixel is itself the same value as the
       current pass, and its four neighbors are too, then it is
       incremented. If no pixels are incremented by the end of the
       pass, then we go again. Edge pixels are counted as always
       excluded (this should be true anyway for any sane mask, but if
       it isn't this will ensure that we eventually exit). */
    while (1) {
        // If this doesn't get set by the end of this pass, then we're done.
        int has_anything_changed = 0;
        uint8_t *current_pixel0 = data + 1 + linesize, *current_pixel;
        current_pass++;

        for (y = 1; y < h - 1; y++) {
            current_pixel = current_pixel0;
            for (x = 1; x < w - 1; x++) {
                /* Apply the in-place erosion transform. It is based
                   on the following two premises:
                   1 - Any pixel that fails 1 erosion will fail all
                       future erosions.
                   2 - Only pixels having survived all erosions up to
                       the present will be >= to current_pass.
                   It doesn't matter if it survived the current pass,
                   failed it, or hasn't been tested yet.  By using >=
                   instead of ==, we allow the algorithm to work in
                   place. */
                if (*current_pixel >= current_pass &&
                    *(current_pixel + 1) >= current_pass &&
                    *(current_pixel - 1) >= current_pass &&
                    *(current_pixel + linesize) >= current_pass &&
                    *(current_pixel - linesize) >= current_pass) {
                    /* Increment the value since it still has not been
                     * eroded, as evidenced by the if statement that
                     * just evaluated to true. */
                    (*current_pixel)++;
                    has_anything_changed = 1;
                }
                current_pixel++;
            }
            current_pixel0 += linesize;
        }
        if (!has_anything_changed) break;
    }

    /* Apply the fudge factor, which will increase the size of the
     * mask a little to reduce jitter at the cost of more blur. */
    for (y = 1; y < h - 1; y++)
        for (x = 1; x < w - 1; x++)
            data[(y * linesize) + x] =
                apply_mask_fudge_factor(data[(y * linesize) + x]);

    /* As a side-effect, we now know the maximum mask size, which
     * we'll use to generate our masks.
     * Apply the fudge factor to this number too, since we must ensure
     * that enough masks are generated. */
    *max_mask_size = apply_mask_fudge_factor(current_pass + 1);
}

/**
 * Generate a scaled down image with half width, height, and intensity.
 *
 * This function not only scales down an image, but halves the value
 * in each pixel too. The purpose of this is to produce a chroma
 * filter image out of a luma filter image. The pixel values store the
 * distance to the edge of the logo and halving the dimensions halves
 * the distance. This function rounds up, because a downwards rounding
 * error could cause the filter to fail, but an upwards rounding error
 * will only cause a minor amount of excess blur in the chroma planes.
 */
static void generate_half_size_image(const uint8_t *src_data, int src_stride,
                                     uint8_t *dst_data, int dst_stride,
                                     int src_w, int src_h, int *max_mask_size) {
    int x, y;

    /* Copy over the image data, using the average of 4 pixels for to
     * calculate each downsampled pixel. */
    for (y = 0; y < src_h / 2; y++) {
        for (x = 0; x < src_w / 2; x++) {
            /* Set the pixel if there exists a non-zero value in the
             * source pixels, else clear it. */
            dst_data[(y * dst_stride) + x] =
                src_data[((y << 1) * src_stride) + (x << 1)] ||
                src_data[((y << 1) * src_stride) + (x << 1) + 1] ||
                src_data[(((y << 1) + 1) * src_stride) + (x << 1)] ||
                src_data[(((y << 1) + 1) * src_stride) + (x << 1) + 1];
            dst_data[(y * dst_stride) + x] =
                VSMIN(1, dst_data[(y * dst_stride) + x]);
        }
    }

    convert_mask_to_strength_mask(dst_data, dst_stride, src_w / 2, src_h / 2, 0,
                                  max_mask_size);
}

/**
 * Blur image.
 *
 * It takes a pixel that is inside the mask and blurs it. It does so
 * by finding the average of all the pixels within the mask and
 * outside of the mask.
 *
 * @param mask the filter mask
 * @param mask_data the mask plane to use for averaging
 * @param image_data the image plane to blur
 * @param image_linesize the length of image lines
 * @param w width of the image
 * @param h height of the image
 * @param x x-coordinate of the pixel to blur
 * @param y y-coordinate of the pixel to blur
 */
static unsigned int blur_pixel(int ***mask, const uint8_t *mask_data,
                               uint8_t *image_data, int image_linesize, int w,
                               int h, int x, int y) {
    /* Mask size tells how large a circle to use. The radius is about
     * (slightly larger than) mask size. */
    int mask_size;
    int start_posx, start_posy, end_posx, end_posy;
    int i, j;
    unsigned int accumulator = 0, divisor = 0;
    // What pixel we are reading out of the circular blur mask.
    const uint8_t *image_read_position;
    // What pixel we are reading out of the filter image.
    const uint8_t *mask_read_position;

    // Prepare our bounding rectangle and clip it if need be.
    mask_size = mask_data[y * w + x];
    start_posx = VSMAX(0, x - mask_size);
    start_posy = VSMAX(0, y - mask_size);
    end_posx = VSMIN(w - 1, x + mask_size);
    end_posy = VSMIN(h - 1, y + mask_size);

    image_read_position = image_data + image_linesize * start_posy + start_posx;
    mask_read_position = mask_data + w * start_posy + start_posx;

    for (j = start_posy; j <= end_posy; j++) {
        for (i = start_posx; i <= end_posx; i++) {
            /* Check if this pixel is in the mask or not. Only use the
             * pixel if it is not. */
            if (!(*mask_read_position) &&
                mask[mask_size][i - start_posx][j - start_posy]) {
                accumulator += *image_read_position;
                divisor++;
            }

            image_read_position++;
            mask_read_position++;
        }

        image_read_position += (image_linesize - ((end_posx + 1) - start_posx));
        mask_read_position += (w - ((end_posx + 1) - start_posx));
    }

    /* If divisor is 0, it means that not a single pixel is outside of
       the logo, so we have no data.  Else we need to normalise the
       data using the divisor. */
    return divisor == 0
               ? 255
               : (accumulator + (divisor / 2)) / divisor; /* divide, taking into
                                                             account average
                                                             rounding error */
}

/**
 * Blur image plane using a mask.
 *
 * @param mask filter mask
 * @param src_data image to have it's logo removed
 * @param src_stride how far apart (in memory) two consecutive lines are
 * @param dst_data where the output image will be stored
 * @param dst_stride same as src_stride, but for the destination image
 * @param mask_data the mask plane to use for averaging
 * @param w width of the image, same for source and destination
 * @param h height of the image, same for source and destination.
 * @param bbox coordinates that contain at least one logo pixel
 *
 * This function processes an entire plane. Pixels outside of the logo are
 * copied to the output without change, and pixels inside the logo have the
 * de-blurring function applied.
 */
static void blur_image(int ***mask, const uint8_t *src_data, int src_stride,
                       uint8_t *dst_data, int dst_stride,
                       const uint8_t *mask_data, int w, int h,
                       BoundingBox *bbox) {
    int x, y;
    uint8_t *dst_line;
    const uint8_t *src_line;

    for (y = bbox->y1; y <= bbox->y2; y++) {
        src_line = src_data + src_stride * y;
        dst_line = dst_data + dst_stride * y;

        for (x = bbox->x1; x <= bbox->x2; x++) {
            if (mask_data[y * w + x]) {
                // Only process if we are in the mask.
                dst_line[x] = blur_pixel(mask, mask_data, dst_data, dst_stride,
                                         w, h, x, y);
            } else {
                // Else just copy the data.
                dst_line[x] = src_line[x];
            }
        }
    }
}

static void VS_CC removeLogoInit(VSMap *in, VSMap *out, void **instanceData,
                                 VSNode *node, VSCore *core,
                                 const VSAPI *vsapi) {
    RemoveLogoData *d = (RemoveLogoData *)*instanceData;
    vsapi->setVideoInfo(d->vi, 1, node);
}

static const VSFrameRef *VS_CC removeLogoGetFrame(
    int n, int activationReason, void **instanceData, void **frameData,
    VSFrameContext *frameCtx, VSCore *core, const VSAPI *vsapi) {
    RemoveLogoData *d = (RemoveLogoData *)*instanceData;

    if (activationReason == arInitial) {
        vsapi->requestFrameFilter(n, d->node, frameCtx);
    } else if (activationReason == arAllFramesReady) {
        const VSFrameRef *src = vsapi->getFrameFilter(n, d->node, frameCtx);

        VSFrameRef *dst = vsapi->copyFrame(src, core);

        blur_image(d->mask, vsapi->getReadPtr(src, 0), vsapi->getStride(src, 0),
                   vsapi->getWritePtr(dst, 0), vsapi->getStride(dst, 0),
                   d->full_mask_data, d->vi->width, d->vi->height,
                   &d->full_mask_bbox);
        blur_image(d->mask, vsapi->getReadPtr(src, 1), vsapi->getStride(src, 1),
                   vsapi->getWritePtr(dst, 1), vsapi->getStride(dst, 1),
                   d->half_mask_data, d->vi->width / 2, d->vi->height / 2,
                   &d->half_mask_bbox);
        blur_image(d->mask, vsapi->getReadPtr(src, 2), vsapi->getStride(src, 2),
                   vsapi->getWritePtr(dst, 2), vsapi->getStride(dst, 2),
                   d->half_mask_data, d->vi->width / 2, d->vi->height / 2,
                   &d->half_mask_bbox);

        vsapi->freeFrame(src);

        return dst;
    }

    return 0;
}

// Free all allocated data on filter destruction
static void VS_CC removeLogoFree(void *instanceData, VSCore *core,
                                 const VSAPI *vsapi) {
    RemoveLogoData *d = (RemoveLogoData *)instanceData;
    int a, b;

    VS_ALIGNED_FREE(d->full_mask_data);
    VS_ALIGNED_FREE(d->half_mask_data);

    if (d->mask) {
        // Loop through each mask.
        for (a = 0; a <= d->max_mask_size; a++) {
            // Loop through each scanline in a mask.
            for (b = -a; b <= a; b++) {
                VS_ALIGNED_FREE(d->mask[a][b + a]);  // Free a scanline.
            }
            VS_ALIGNED_FREE(d->mask[a]);
        }
        // Free the array of pointers pointing to the masks.
        VS_ALIGNED_FREE(d->mask);
    }
    vsapi->freeNode(d->node);
    free(d);
}

static void VS_CC removeLogoCreate(const VSMap *in, VSMap *out, void *userData,
                                   VSCore *core, const VSAPI *vsapi) {
    RemoveLogoData d;
    RemoveLogoData *data;

    d.node = vsapi->propGetNode(in, "clip", 0, 0);
    d.vi = vsapi->getVideoInfo(d.node);

    // only handle 8bit integer formats
    if (!isConstantFormat(d.vi) || d.vi->format->sampleType != stInteger ||
        d.vi->format->bitsPerSample != 8) {
        vsapi->setError(
            out,
            "RemoveLogo: only constant format 8bit integer input supported");
        vsapi->freeNode(d.node);
        return;
    }

    int err;
    const char *filename = vsapi->propGetData(in, "filename", 0, &err);
    if (err) {
        vsapi->setError(out, "RemoveLogo: the bitmap file name is mandatory");
        vsapi->freeNode(d.node);
        return;
    }

    int ***mask;
    int a, b, c, w, h;
    int full_max_mask_size, half_max_mask_size;

    // Load our mask image.
    if (load_mask(&d.full_mask_data, &w, &h, filename, out, core, vsapi) < 0) {
        vsapi->freeNode(d.node);
        return;
    }

    convert_mask_to_strength_mask(d.full_mask_data, w, w, h, 16,
                                  &full_max_mask_size);

    // Create the scaled down mask image for the chroma planes.
    VS_ALIGNED_MALLOC(&d.half_mask_data, w / 2 * h / 2, ALIGN);
    if (!d.half_mask_data) {
        vsapi->setError(out, "RemoveLogo: no memory");
        vsapi->freeNode(d.node);
        return;
    }
    generate_half_size_image(d.full_mask_data, w, d.half_mask_data, w / 2, w, h,
                             &half_max_mask_size);

    d.max_mask_size = VSMAX(full_max_mask_size, half_max_mask_size);

    /* Create a circular mask for each size up to max_mask_size. When
       the filter is applied, the mask size is determined on a pixel
       by pixel basis, with pixels nearer the edge of the logo getting
       smaller mask sizes. */
    VS_ALIGNED_MALLOC(&mask, (d.max_mask_size + 1) * sizeof(int **), ALIGN);
    if (!mask) {
        vsapi->setError(out, "RemoveLogo: no memory");
        vsapi->freeNode(d.node);
        return;
    }

    for (a = 0; a <= d.max_mask_size; a++) {
        VS_ALIGNED_MALLOC(&mask[a], ((a * 2) + 1) * sizeof(int *), ALIGN);
        if (!mask[a]) {
            vsapi->setError(out, "RemoveLogo: no memory");
            vsapi->freeNode(d.node);
            return;
        }
        for (b = -a; b <= a; b++) {
            VS_ALIGNED_MALLOC(&mask[a][b + a], ((a * 2) + 1) * sizeof(int),
                              ALIGN);
            if (!mask[a][b + a]) {
                vsapi->setError(out, "RemoveLogo: no memory");
                vsapi->freeNode(d.node);
                return;
            }
            for (c = -a; c <= a; c++) {
                if ((b * b) + (c * c) <= (a * a))  // Circular 0/1 mask.
                    mask[a][b + a][c + a] = 1;
                else
                    mask[a][b + a][c + a] = 0;
            }
        }
    }
    d.mask = mask;

    /* Calculate our bounding rectangles, which determine in what
     * region the logo resides for faster processing. */
    calculate_bounding_box(&d.full_mask_bbox, d.full_mask_data, w, h);
    calculate_bounding_box(&d.half_mask_bbox, d.half_mask_data, w / 2, h / 2);

    data = malloc(sizeof(d));
    *data = d;

    vsapi->createFilter(in, out, "RemoveLogo", removeLogoInit,
                        removeLogoGetFrame, removeLogoFree, fmParallel, 0, data,
                        core);
}

VS_EXTERNAL_API(void)
VapourSynthPluginInit(VSConfigPlugin configFunc,
                      VSRegisterFunction registerFunc, VSPlugin *plugin) {
    configFunc("xyz.noctem.removelogo", "rmlogo",
               "Remove a TV logo based on a mask image.",
               VAPOURSYNTH_API_VERSION, 1, plugin);
    registerFunc("RemoveLogo", "clip:clip;filename:data;", removeLogoCreate, 0,
                 plugin);
}
