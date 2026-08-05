/****************************************************************************
 * apps/examples/camcap/camcap_main.c
 * Capture one frame from /dev/video0 and save to file.
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <nuttx/video/video.h>

#define CAP_DEV   "/dev/video0"
#define OUT_FILE  "/data/capture.rgb565"
#define CAP_W     1024
#define CAP_H     600
#define CAP_FMT   V4L2_PIX_FMT_RGB565
#define BUF_NUM   2
#define BUF_ALIGN 64

static int xioctl(int fd, int req, void *arg)
{
  int r;
  do { r = ioctl(fd, req, arg); } while (r == -1 && errno == EINTR);
  return r;
}

int main(int argc, char *argv[])
{
  int fd = -1, outfd = -1, ret = EXIT_FAILURE;
  struct v4l2_format fmt;
  struct v4l2_requestbuffers rb;
  struct v4l2_buffer buf;
  unsigned int i;
  void *bufs[BUF_NUM];
  size_t framesize;

  printf("camcap: capturing %dx%d from %s\n", CAP_W, CAP_H, CAP_DEV);

  /* 1. Open camera */

  fd = open(CAP_DEV, O_RDWR, 0);
  if (fd < 0) { printf("ERROR: open %s: %d\n", CAP_DEV, errno); goto out; }

  /* 2. Set format */

  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.pixelformat = CAP_FMT;
  fmt.fmt.pix.width  = CAP_W;
  fmt.fmt.pix.height = CAP_H;
  if (xioctl(fd, VIDIOC_S_FMT, &fmt) < 0)
    { printf("ERROR: S_FMT: %d\n", errno); goto out; }
  printf("  format: %dx%d sizeimage=%u\n",
         fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
  framesize = fmt.fmt.pix.sizeimage;

  /* 3. Request buffers */

  memset(&rb, 0, sizeof(rb));
  rb.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  rb.memory = V4L2_MEMORY_MMAP;
  rb.count  = BUF_NUM;
  if (xioctl(fd, VIDIOC_REQBUFS, &rb) < 0)
    { printf("ERROR: REQBUFS: %d\n", errno); goto out; }
  printf("  buffers: %u\n", rb.count);

  /* 4. Map & queue buffers */

  for (i = 0; i < rb.count; i++)
    {
      memset(&buf, 0, sizeof(buf));
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index  = i;
      if (xioctl(fd, VIDIOC_QUERYBUF, &buf) < 0)
        { printf("ERROR: QUERYBUF %u: %d\n", i, errno); goto out; }
      bufs[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                     MAP_SHARED, fd, buf.m.offset);
      if (bufs[i] == MAP_FAILED)
        { printf("ERROR: mmap %u: %d\n", i, errno); goto out; }
      if (xioctl(fd, VIDIOC_QBUF, &buf) < 0)
        { printf("ERROR: QBUF %u: %d\n", i, errno); goto out; }
    }

  /* 5. Start streaming */

  {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_STREAMON, &type) < 0)
      { printf("ERROR: STREAMON: %d\n", errno); goto out; }
  }
  printf("  streaming started, waiting for frame...\n");

  /* 6. Dequeue one frame */

  memset(&buf, 0, sizeof(buf));
  buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (xioctl(fd, VIDIOC_DQBUF, &buf) < 0)
    { printf("ERROR: DQBUF: %d\n", errno); goto out; }
  printf("  got frame %u: %u bytes\n", buf.index, buf.bytesused);

  /* 7. Write to file */

  outfd = open(OUT_FILE, O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (outfd < 0)
    { printf("ERROR: open %s: %d\n", OUT_FILE, errno); goto out; }

  {
    ssize_t n = write(outfd, bufs[buf.index], buf.bytesused);
    if (n < 0) { printf("ERROR: write: %d\n", errno); goto out; }
    printf("  wrote %ld bytes to %s\n", (long)n, OUT_FILE);
  }

  /* 8. Re-queue buffer and stop */

  if (xioctl(fd, VIDIOC_QBUF, &buf) < 0)
    printf("WARN: QBUF after capture: %d\n", errno);

  {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd, VIDIOC_STREAMOFF, &type);
  }

  ret = EXIT_SUCCESS;

out:
  if (outfd >= 0) close(outfd);
  if (fd >= 0) close(fd);
  return ret;
}