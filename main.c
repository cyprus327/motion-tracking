#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <SDL2/SDL.h>

#define VIDEO_DEVICE "/dev/video0"

#define WINDOW_W 1280
#define WINDOW_H 720

#define GRID_W 64
#define GRID_H 48
#define CELL_W (WINDOW_W / GRID_W)
#define CELL_H (WINDOW_H / GRID_H)

#define CHANGES_THRESHOLD 26

typedef unsigned char byte;

static inline void convert_to_grayscale(const void* yuyv, byte* gray, int width, int height) {
    const byte* yuyv_data = (const byte*)yuyv;
    for (int i = 0; i < width * height; i++) {
        gray[i] = yuyv_data[i * 2]; // extract y
    }
}

static inline void calculate_aabb(const byte* diff, int width, int height, int* xmin, int* ymin, int* xmax, int* ymax) {
    *xmin = width;
    *ymin = height;
    *xmax = 0;
    *ymax = 0;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (diff[y * width + x] > 0) {
                if (x < *xmin) *xmin = x;
                if (x > *xmax) *xmax = x;
                if (y < *ymin) *ymin = y;
                if (y > *ymax) *ymax = y;
            }
        }
    }
}

int main(int argc, char* argv[]) {
    // open video device
    const int fd = open(VIDEO_DEVICE, O_RDWR);
    if (-1 == fd) {
        perror("Opening video device");
        return 1;
    }

    // query camera capabilities
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
        perror("VIDIOC_QUERYCAP");
        close(fd);
        return 1;
    }

    printf("Driver: %s\n", cap.driver);
    printf("Card: %s\n", cap.card);

    // set the video format to YUYV
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = WINDOW_W;
    fmt.fmt.pix.height = WINDOW_H;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) == -1) {
        perror("VIDIOC_S_FMT");
        close(fd);
        return 1;
    }

    // request buffer for capturing
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 1; // requesting one buffer
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
        perror("VIDIOC_REQBUFS");
        close(fd);
        return 1;
    }

    // map the buffer
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = 0;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) == -1) {
        perror("VIDIOC_QUERYBUF");
        close(fd);
        return 1;
    }

    void* buffer = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (MAP_FAILED == buffer) {
        perror("mmap");
        close(fd);
        return 1;
    }

    // Queue the buffer for capture
    if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
        perror("VIDIOC_QBUF");
        close(fd);
        return 1;
    }

    // start video capture
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
        perror("VIDIOC_STREAMON");
        close(fd);
        return 1;
    }

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("Webcam Capture", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_W, WINDOW_H, SDL_WINDOW_SHOWN);
    if (!window) {
        fprintf(stderr, "SDL_CreateWindow error: %s\n", SDL_GetError());
        close(fd);
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer) {
        fprintf(stderr, "SDL_CreateRenderer error: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        close(fd);
        return 1;
    }

    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_YUY2, SDL_TEXTUREACCESS_STREAMING, WINDOW_W, WINDOW_H);
    if (!texture) {
        fprintf(stderr, "SDL_CreateTexture error: %s\n", SDL_GetError());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        close(fd);
        return 1;
    }

    byte curr_gray[WINDOW_W * WINDOW_H] = {0};
    byte prev_gray[WINDOW_W * WINDOW_H] = {0};
    byte diff[WINDOW_W * WINDOW_H] = {0};

    byte motion_map[GRID_W * GRID_H] = {0};

    SDL_Event e;
    char running = 1;
    while (running) {
        // wait for a new frame
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == -1) {
            perror("VIDIOC_DQBUF");
            break;
        }

        convert_to_grayscale(buffer, curr_gray, WINDOW_W, WINDOW_H);

        // find overall motion aabb
        const int STEP_SIZE = 10;
        for (int y = 0; y < WINDOW_H / STEP_SIZE; y++) {
            const int ay = y * STEP_SIZE;
            for (int x = 0; x < WINDOW_W / STEP_SIZE; x++) {
                const int ax = x * STEP_SIZE;
                const int i = ay * WINDOW_W + ax;
                const int val = abs(curr_gray[i] - prev_gray[i]) > CHANGES_THRESHOLD ? 255 : 0;
                for (int dy = ay; dy < ay + STEP_SIZE; dy++) {
                    for (int dx = ax; dx < ax + STEP_SIZE; dx++) {
                        diff[dy * WINDOW_W + dx] = val;
                    }
                }
            }
        }

        int xmin, ymin, xmax, ymax;
        calculate_aabb(diff, WINDOW_W, WINDOW_H, &xmin, &ymin, &xmax, &ymax);

        // find which grid cells have motion
        for (int gy = 0, i = 0; gy < GRID_H; gy++) {
            for (int gx = 0; gx < GRID_W; gx++, i++) {
                char motion = 0;
                for (int y = gy * CELL_H; y < (gy + 1) * CELL_H; y++) {
                    for (int x = gx * CELL_W; x < (gx + 1) * CELL_W; x++) {
                        const int ind = y * WINDOW_W + x;
                        if (abs(curr_gray[ind] - prev_gray[ind]) > CHANGES_THRESHOLD) {
                            motion = 1;
                            break;
                        }
                    }
                    if (motion) {
                        break;
                    }
                }

                motion_map[i] = motion;
            }
        }

        SDL_UpdateTexture(texture, NULL, buffer, fmt.fmt.pix.width * 2);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, NULL, NULL);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        SDL_Rect aabb = {xmin, ymin, xmax - xmin, ymax - ymin};
        SDL_RenderDrawRect(renderer, &aabb);

        SDL_SetRenderDrawColor(renderer, 0, 255, 0, 255);
        for (int gy = 0, i = 0; gy < GRID_H; gy++) {
            for (int gx = 0; gx < GRID_W; gx++, i++) {
                if (motion_map[i]) {
                    const SDL_Rect rect = { gx * CELL_W, gy * CELL_H, CELL_W, CELL_H };
                    SDL_RenderDrawRect(renderer, &rect);
                }
            }
        }

        SDL_RenderPresent(renderer);

        memcpy(prev_gray, curr_gray, WINDOW_W * WINDOW_H);

        // requeue buffer for the next frame
        if (ioctl(fd, VIDIOC_QBUF, &buf) == -1) {
            perror("VIDIOC_QBUF");
            break;
        }

        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running = 0;
            }
        }
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    munmap(buffer, buf.length);
    close(fd);

    return 0;
}
