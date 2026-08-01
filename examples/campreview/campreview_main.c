/****************************************************************************
 * apps/examples/campreview/campreview_main.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <malloc.h>

#include <sys/ioctl.h>
#include <sys/mman.h>

#include <nuttx/video/fb.h>
#include <nuttx/video/video.h>
#include <nuttx/video/v4l2_cap.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAMPREVIEW_VIDEO_DEV  "/dev/video0"
#define CAMPREVIEW_FB_DEV     "/dev/fb0"

/* Two capture buffers so the sensor can fill one while we convert the
 * other.  More would only add latency, the panel cannot show them.
 */

#define CAMPREVIEW_BUFNUM     2

/* Buffer pointers handed to VIDIOC_QBUF must be at least 32-byte aligned.
 * Use a full 64-byte cache line, which is what the CSI driver aligns its
 * own DMA buffers to, so that a DMA write never shares a cache line with
 * anything else.
 */

#define CAMPREVIEW_BUFALIGN   64

/* Print a progress line every N frames instead of once per frame. */

#define CAMPREVIEW_LOG_PERIOD 60

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct campreview_buf_s
{
  FAR uint8_t *start;
  uint32_t     length;
};

struct campreview_state_s
{
  int    fb_fd;
  int    v_fd;
  bool   streaming;

  FAR uint8_t *fbmem;                  /* mmap'ed framebuffer */
  size_t       fblen;

  struct fb_videoinfo_s vinfo;
  struct fb_planeinfo_s pinfo;

  struct campreview_buf_s bufs[CAMPREVIEW_BUFNUM];
  uint8_t nbufs;                       /* Buffers actually allocated */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_campreview_stop;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: campreview_usage
 ****************************************************************************/

static void campreview_usage(FAR const char *progname)
{
  printf("Usage: %s [nframes]\n", progname);
  printf("       %s -h\n", progname);
  printf("\n");
  printf("Capture RGB565 frames from %s and render them into the\n",
         CAMPREVIEW_VIDEO_DEV);
  printf("RGB888 framebuffer %s.\n", CAMPREVIEW_FB_DEV);
  printf("\n");
  printf("  nframes  Number of frames to preview.  0 (the default) means\n");
  printf("           run until an error occurs or SIGINT is received.\n");
}

/****************************************************************************
 * Name: campreview_sigint
 ****************************************************************************/

static void campreview_sigint(int signo)
{
  g_campreview_stop = true;
}

/****************************************************************************
 * Name: campreview_now_ms
 ****************************************************************************/

static uint64_t campreview_now_ms(void)
{
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000 + (uint64_t)(ts.tv_nsec / 1000000);
}

/****************************************************************************
 * Name: campreview_convert
 *
 * Description:
 *   Expand one RGB565 frame into the RGB888 framebuffer.
 *
 *   RGB565 source words are 16-bit little-endian, laid out as
 *   RRRRRGGG GGGBBBBB.  The DSI framebuffer stores the three components in
 *   R, G, B byte order (the display self-test writes fb[0]=0xff to get
 *   red).  Each component is expanded by bit replication so that a
 *   full-scale source value maps to 0xff.
 *
 *   No scaling is done: the caller has already verified that the capture
 *   and panel resolutions match.
 *
 ****************************************************************************/

static void campreview_convert(FAR const uint8_t *src, FAR uint8_t *dst,
                               uint32_t npixels)
{
  FAR const uint16_t *s = (FAR const uint16_t *)src;
  FAR uint8_t *d = dst;
  uint32_t i;

  for (i = 0; i < npixels; i++)
    {
      uint16_t pix = *s++;
      uint8_t r5 = (uint8_t)(pix >> 11);
      uint8_t g6 = (uint8_t)((pix >> 5) & 0x3f);
      uint8_t b5 = (uint8_t)(pix & 0x1f);

      *d++ = (uint8_t)((r5 << 3) | (r5 >> 2));
      *d++ = (uint8_t)((g6 << 2) | (g6 >> 4));
      *d++ = (uint8_t)((b5 << 3) | (b5 >> 2));
    }
}

/****************************************************************************
 * Name: campreview_free_bufs
 ****************************************************************************/

static void campreview_free_bufs(FAR struct campreview_state_s *st)
{
  uint8_t i;

  for (i = 0; i < st->nbufs; i++)
    {
      free(st->bufs[i].start);
      st->bufs[i].start  = NULL;
      st->bufs[i].length = 0;
    }

  st->nbufs = 0;
}

/****************************************************************************
 * Name: campreview_fb_open
 *
 * Description:
 *   Open the framebuffer, query its geometry and map it into the process.
 *   mmap() is used rather than pinfo.fbmem directly because that is what
 *   apps/examples/fb does: in a KERNEL build only mmap() returns an address
 *   the application may touch.
 *
 ****************************************************************************/

static int campreview_fb_open(FAR struct campreview_state_s *st)
{
  st->fb_fd = open(CAMPREVIEW_FB_DEV, O_RDWR);
  if (st->fb_fd < 0)
    {
      printf("campreview: ERROR: open %s failed: %d\n",
             CAMPREVIEW_FB_DEV, errno);
      return -errno;
    }

  if (ioctl(st->fb_fd, FBIOGET_VIDEOINFO,
            (unsigned long)((uintptr_t)&st->vinfo)) < 0)
    {
      printf("campreview: ERROR: FBIOGET_VIDEOINFO failed: %d\n", errno);
      return -errno;
    }

  if (ioctl(st->fb_fd, FBIOGET_PLANEINFO,
            (unsigned long)((uintptr_t)&st->pinfo)) < 0)
    {
      printf("campreview: ERROR: FBIOGET_PLANEINFO failed: %d\n", errno);
      return -errno;
    }

  printf("campreview: fb %ux%u fmt=%u bpp=%u stride=%u fblen=%zu\n",
         st->vinfo.xres, st->vinfo.yres, st->vinfo.fmt,
         st->pinfo.bpp, st->pinfo.stride, st->pinfo.fblen);

  if (st->vinfo.fmt != FB_FMT_RGB24 || st->pinfo.bpp != 24)
    {
      printf("campreview: ERROR: need FB_FMT_RGB24 (%d) at 24bpp, "
             "got fmt=%u bpp=%u\n",
             FB_FMT_RGB24, st->vinfo.fmt, st->pinfo.bpp);
      return -ENOTSUP;
    }

  if (st->vinfo.xres == 0 || st->vinfo.yres == 0 ||
      st->pinfo.stride < (unsigned)st->vinfo.xres * 3 ||
      st->pinfo.fblen < (size_t)st->pinfo.stride * st->vinfo.yres)
    {
      printf("campreview: ERROR: inconsistent fb geometry\n");
      return -EINVAL;
    }

  st->fblen = st->pinfo.fblen;
  st->fbmem = mmap(NULL, st->fblen, PROT_READ | PROT_WRITE,
                   MAP_SHARED | MAP_FILE, st->fb_fd, 0);
  if (st->fbmem == MAP_FAILED)
    {
      printf("campreview: ERROR: mmap failed: %d\n", errno);
      st->fbmem = NULL;
      return -errno;
    }

  return OK;
}

/****************************************************************************
 * Name: campreview_video_open
 *
 * Description:
 *   Open the capture device, set RGB565 at the panel resolution, allocate
 *   and queue the user-pointer buffers, then start streaming.
 *
 *   V4L2_MEMORY_USERPTR with V4L2_BUF_MODE_RING mirrors
 *   apps/examples/camera/camera_main.c, which is the pattern known to work
 *   with this driver stack.
 *
 ****************************************************************************/

static int campreview_video_open(FAR struct campreview_state_s *st)
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  struct v4l2_requestbuffers req;
  struct v4l2_format fmt;
  struct v4l2_buffer buf;
  uint32_t framesize;
  uint8_t i;

  framesize = (uint32_t)st->vinfo.xres * st->vinfo.yres * 2;

  st->v_fd = open(CAMPREVIEW_VIDEO_DEV, O_RDWR);
  if (st->v_fd < 0)
    {
      printf("campreview: ERROR: open %s failed: %d\n",
             CAMPREVIEW_VIDEO_DEV, errno);
      return -errno;
    }

  memset(&fmt, 0, sizeof(fmt));
  fmt.type                = type;
  fmt.fmt.pix.width       = st->vinfo.xres;
  fmt.fmt.pix.height      = st->vinfo.yres;
  fmt.fmt.pix.field       = V4L2_FIELD_ANY;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;

  if (ioctl(st->v_fd, VIDIOC_S_FMT, (uintptr_t)&fmt) < 0)
    {
      printf("campreview: ERROR: VIDIOC_S_FMT RGB565 %ux%u failed: %d\n",
             st->vinfo.xres, st->vinfo.yres, errno);
      return -errno;
    }

  memset(&req, 0, sizeof(req));
  req.type   = type;
  req.memory = V4L2_MEMORY_USERPTR;
  req.count  = CAMPREVIEW_BUFNUM;
  req.mode   = V4L2_BUF_MODE_RING;

  if (ioctl(st->v_fd, VIDIOC_REQBUFS, (uintptr_t)&req) < 0)
    {
      printf("campreview: ERROR: VIDIOC_REQBUFS failed: %d\n", errno);
      return -errno;
    }

  for (i = 0; i < CAMPREVIEW_BUFNUM; i++)
    {
      st->bufs[i].start = memalign(CAMPREVIEW_BUFALIGN, framesize);
      if (st->bufs[i].start == NULL)
        {
          printf("campreview: ERROR: out of memory for buffer %d (%lu B)\n",
                 i, (unsigned long)framesize);
          return -ENOMEM;
        }

      st->bufs[i].length = framesize;
      st->nbufs          = i + 1;
    }

  for (i = 0; i < st->nbufs; i++)
    {
      memset(&buf, 0, sizeof(buf));
      buf.type      = type;
      buf.memory    = V4L2_MEMORY_USERPTR;
      buf.index     = i;
      buf.m.userptr = (uintptr_t)st->bufs[i].start;
      buf.length    = st->bufs[i].length;

      if (ioctl(st->v_fd, VIDIOC_QBUF, (uintptr_t)&buf) < 0)
        {
          printf("campreview: ERROR: VIDIOC_QBUF %d failed: %d\n",
                 i, errno);
          return -errno;
        }
    }

  if (ioctl(st->v_fd, VIDIOC_STREAMON, (uintptr_t)&type) < 0)
    {
      printf("campreview: ERROR: VIDIOC_STREAMON failed: %d\n", errno);
      return -errno;
    }

  st->streaming = true;
  return OK;
}

/****************************************************************************
 * Name: campreview_cleanup
 ****************************************************************************/

static void campreview_cleanup(FAR struct campreview_state_s *st)
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (st->streaming)
    {
      ioctl(st->v_fd, VIDIOC_STREAMOFF, (uintptr_t)&type);
      st->streaming = false;
    }

  campreview_free_bufs(st);

  if (st->fbmem != NULL)
    {
      munmap(st->fbmem, st->fblen);
      st->fbmem = NULL;
    }

  if (st->v_fd >= 0)
    {
      close(st->v_fd);
      st->v_fd = -1;
    }

  if (st->fb_fd >= 0)
    {
      close(st->fb_fd);
      st->fb_fd = -1;
    }
}

/****************************************************************************
 * Name: campreview_loop
 *
 * Description:
 *   Dequeue, convert, hand the framebuffer to the display, requeue.
 *
 *   FBIOPAN_DISPLAY is what reaches the driver's pandisplay hook, which
 *   performs the cache writeback the DSI DMA needs.  FBIO_UPDATE is not
 *   usable here: it only exists when CONFIG_FB_UPDATE is enabled and it
 *   routes to the updatearea hook, which this framebuffer does not
 *   implement.  Without the writeback nothing would ever appear on screen.
 *
 ****************************************************************************/

static int campreview_loop(FAR struct campreview_state_s *st,
                           unsigned long nframes)
{
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  unsigned long framecnt = 0;
  uint64_t mark;
  bool panwarned = false;
  uint32_t rowpixels = st->vinfo.xres;
  uint32_t nrows = st->vinfo.yres;
  uint32_t srcstride = rowpixels * 2;
  int ret = OK;

  mark = campreview_now_ms();

  while (!g_campreview_stop && (nframes == 0 || framecnt < nframes))
    {
      struct v4l2_buffer buf;
      FAR const uint8_t *src;
      FAR uint8_t *dst;
      uint32_t row;

      memset(&buf, 0, sizeof(buf));
      buf.type   = type;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (ioctl(st->v_fd, VIDIOC_DQBUF, (uintptr_t)&buf) < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }

          printf("campreview: ERROR: VIDIOC_DQBUF failed: %d\n", errno);
          return -errno;
        }

      /* Convert row by row so that a framebuffer stride wider than
       * xres * 3 is handled correctly.
       */

      src = (FAR const uint8_t *)(uintptr_t)buf.m.userptr;
      dst = st->fbmem;

      for (row = 0; row < nrows; row++)
        {
          campreview_convert(src, dst, rowpixels);
          src += srcstride;
          dst += st->pinfo.stride;
        }

      /* Hand the framebuffer to the display.  This is the cache
       * writeback; skipping it leaves the DMA on stale data.
       */

      if (ioctl(st->fb_fd, FBIOPAN_DISPLAY,
                (unsigned long)((uintptr_t)&st->pinfo)) < 0 && !panwarned)
        {
          printf("campreview: WARNING: FBIOPAN_DISPLAY failed: %d\n",
                 errno);
          panwarned = true;
        }

      /* Recycle the capture buffer. */

      if (ioctl(st->v_fd, VIDIOC_QBUF, (uintptr_t)&buf) < 0)
        {
          printf("campreview: ERROR: VIDIOC_QBUF failed: %d\n", errno);
          return -errno;
        }

      framecnt++;

      if ((framecnt % CAMPREVIEW_LOG_PERIOD) == 0)
        {
          uint64_t now = campreview_now_ms();
          uint64_t elapsed = now - mark;
          unsigned fps10 = 0;

          if (elapsed > 0)
            {
              fps10 = (unsigned)((CAMPREVIEW_LOG_PERIOD * 10000ull) /
                                 elapsed);
            }

          printf("campreview: %lu frames, %u.%u fps\n",
                 framecnt, fps10 / 10, fps10 % 10);
          mark = now;
        }
    }

  printf("campreview: stopped after %lu frames\n", framecnt);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  struct campreview_state_s st;
  unsigned long nframes = 0;
  int ret;

  if (argc > 1)
    {
      if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0)
        {
          campreview_usage(argv[0]);
          return EXIT_SUCCESS;
        }

      nframes = strtoul(argv[1], NULL, 0);
    }

  memset(&st, 0, sizeof(st));
  st.fb_fd = -1;
  st.v_fd  = -1;

  g_campreview_stop = false;
  signal(SIGINT, campreview_sigint);

  ret = campreview_fb_open(&st);
  if (ret < 0)
    {
      campreview_cleanup(&st);
      return EXIT_FAILURE;
    }

  ret = campreview_video_open(&st);
  if (ret < 0)
    {
      campreview_cleanup(&st);
      return EXIT_FAILURE;
    }

  printf("campreview: preview %ux%u RGB565 -> RGB888, %s\n",
         st.vinfo.xres, st.vinfo.yres,
         nframes == 0 ? "until stopped" : "limited run");

  ret = campreview_loop(&st, nframes);

  campreview_cleanup(&st);
  return ret < 0 ? EXIT_FAILURE : EXIT_SUCCESS;
}
