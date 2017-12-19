/*
 * Copyright (C) 2016 Rob Clark <robclark@freedesktop.org>
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#define _XOPEN_SOURCE 500
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <assert.h>
#include <ctype.h>
#include <err.h>
#include <fcntl.h>
#include <ftw.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <curses.h>
#include <libconfig.h>

#include <freedreno_drmif.h>
#include <freedreno_ringbuffer.h>

#include "rnndec.h"


#define MAX_CNTR_PER_GROUP 12

/* NOTE first counter group should always be CP, since we unconditionally
 * use CP counter to measure the gpu freq.
 */
struct counter_group {
	const char *name;
	const char *countable;    /* enum name, ex: "a3xx_cp_perfcounter_select" */
	int ncounters;
	struct {
		const char *select;   /* reg name, ex: "CP_PERFCOUNTER_SELECT" */
		const char *val_hi;   /* reg name, ex: "RBBM_PERFCTR_CP_0_HI" */
		const char *val_lo;   /* reg name, ex: "RBBM_PERFCTR_CP_0_LO" */
		const char *enable;   /* optional reg name, ex: "VBIF_PERF_PWR_CNT_EN" */
		const char *clear;    /* optional reg name */
	} counter[MAX_CNTR_PER_GROUP];
	struct {
		uint16_t select_off;
		uint16_t select_val;
		uint16_t enable_off;
		uint16_t clear_off;
		volatile uint32_t *val_hi;
		volatile uint32_t *val_lo;
	} reg[MAX_CNTR_PER_GROUP];
	/* last sample time: */
	uint32_t stime[MAX_CNTR_PER_GROUP];
	/* for now just care about the low 32b value.. at least then we don't
	 * have to really care that we can't sample both hi and lo regs at the
	 * same time:
	 */
	uint32_t last[MAX_CNTR_PER_GROUP];
	/* current value, ie. by how many did the counter increase in last
	 * sampling period divided by the sampling period:
	 */
	float current[MAX_CNTR_PER_GROUP];
	/* name of currently selected counters (for UI): */
	const char *label[MAX_CNTR_PER_GROUP];
};

static struct {
	char *dtnode;
	uint64_t base;
	uint32_t size;
	void *io;
	uint32_t chipid;
	uint32_t min_freq;
	uint32_t max_freq;
	/* per-generation table of counters: */
	unsigned ngroups;
	struct counter_group *groups;
	/* rnndb: */
	struct rnndeccontext *ctx;
	struct rnndomain *dom;
	/* drm device (for writing select regs via ring): */
	struct fd_device *dev;
	struct fd_pipe *pipe;
	struct fd_ringbuffer *ring;
} dev;

#define ARRAY_SIZE(a) (sizeof (a) / sizeof *(a))
#define MIN2(a, b)    ((a) < (b) ? (a) : (b))
#define MAX2(a, b)    ((a) > (b) ? (a) : (b))

static void config_save(void);
static void config_restore(void);

/*
 * helpers
 */

#define CHUNKSIZE 32

static void *
readfile(const char *path, int *sz)
{
	char *buf = NULL;
	int fd, ret, n = 0;

	fd = open(path, O_RDONLY);
	if (fd < 0)
		return NULL;

	while (1) {
		buf = realloc(buf, n + CHUNKSIZE);
		ret = read(fd, buf + n, CHUNKSIZE);
		if (ret < 0) {
			free(buf);
			*sz = 0;
			return NULL;
		} else if (ret < CHUNKSIZE) {
			n += ret;
			*sz = n;
			return buf;
		} else {
			n += CHUNKSIZE;
		}
	}
}

static uint32_t
gettime_us(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec * 1000000) + (ts.tv_nsec / 1000);
}

static uint32_t
delta(uint32_t a, uint32_t b)
{
	/* deal with rollover: */
	if (a > b)
		return 0xffffffff - a + b;
	else
		return b - a;
}

#define CP_WAIT_FOR_IDLE 38
#define CP_TYPE0_PKT 0x00000000
#define CP_TYPE3_PKT 0xc0000000
#define CP_TYPE4_PKT 0x40000000
#define CP_TYPE7_PKT 0x70000000

static inline void
OUT_RING(struct fd_ringbuffer *ring, uint32_t data)
{
	*(ring->cur++) = data;
}

static inline void
OUT_PKT0(struct fd_ringbuffer *ring, uint16_t regindx, uint16_t cnt)
{
	OUT_RING(ring, CP_TYPE0_PKT | ((cnt-1) << 16) | (regindx & 0x7FFF));
}

static inline void
OUT_PKT3(struct fd_ringbuffer *ring, uint8_t opcode, uint16_t cnt)
{
	OUT_RING(ring, CP_TYPE3_PKT | ((cnt-1) << 16) | ((opcode & 0xFF) << 8));
}


/*
 * Starting with a5xx, pkt4/pkt7 are used instead of pkt0/pkt3
 */

static inline unsigned
_odd_parity_bit(unsigned val)
{
	/* See: http://graphics.stanford.edu/~seander/bithacks.html#ParityParallel
	 * note that we want odd parity so 0x6996 is inverted.
	 */
	val ^= val >> 16;
	val ^= val >> 8;
	val ^= val >> 4;
	val &= 0xf;
	return (~0x6996 >> val) & 1;
}

static inline void
OUT_PKT4(struct fd_ringbuffer *ring, uint16_t regindx, uint16_t cnt)
{
	OUT_RING(ring, CP_TYPE4_PKT | cnt |
			(_odd_parity_bit(cnt) << 7) |
			((regindx & 0x3ffff) << 8) |
			((_odd_parity_bit(regindx) << 27)));
}

static inline void
OUT_PKT7(struct fd_ringbuffer *ring, uint8_t opcode, uint16_t cnt)
{
	OUT_RING(ring, CP_TYPE7_PKT | cnt |
			(_odd_parity_bit(cnt) << 15) |
			((opcode & 0x7f) << 16) |
			((_odd_parity_bit(opcode) << 23)));
}

static struct rnndelem *
regelem(struct rnndomain *domain, const char *name)
{
	int i;
	for (i = 0; i < domain->subelemsnum; i++) {
		struct rnndelem *elem = domain->subelems[i];
		if (!strcmp(elem->name, name))
			return elem;
	}
	return NULL;
}

static const char *
enumname(const char *countable, unsigned n)
{
	static const char *unk[] = { /* this is a bit horrible */
		"unk00", "unk01", "unk02", "unk03", "unk04", "unk05", "unk06", "unk07",
		"unk08", "unk09", "unk0a", "unk0b", "unk0c", "unk0d", "unk0e", "unk0f",
		"unk10", "unk11", "unk12", "unk13", "unk14", "unk15", "unk16", "unk17",
		"unk18", "unk19", "unk1a", "unk1b", "unk1c", "unk1d", "unk1e", "unk1f",
		"unk20", "unk21", "unk22", "unk23", "unk24", "unk25", "unk26", "unk27",
		"unk28", "unk29", "unk2a", "unk2b", "unk2c", "unk2d", "unk2e", "unk2f",
	};
	const char *name;

	name = rnndec_decode_enum(dev.ctx, countable, n);
	if (!name)
		name = unk[n];

	return name;
}

/*
 * code to find stuff in /proc/device-tree:
 */

static void *
readdt(const char *node)
{
	char *path;
	void *buf;
	int sz;

	asprintf(&path, "%s/%s", dev.dtnode, node);
	buf = readfile(path, &sz);
	free(path);

	return buf;
}

static int
find_freqs_fn(const char *fpath, const struct stat *sb, int typeflag, struct FTW *ftwbuf)
{
	const char *fname = fpath + ftwbuf->base;
	int sz;

	if (strcmp(fname, "qcom,gpu-freq") == 0) {
		uint32_t *buf = readfile(fpath, &sz);
		uint32_t freq = ntohl(buf[0]);
		free(buf);
		dev.max_freq = MAX2(dev.max_freq, freq);
		dev.min_freq = MIN2(dev.min_freq, freq);
	}

	return 0;
}

static void
find_freqs(void)
{
	char *path;
	int ret;

	dev.min_freq = ~0;
	dev.max_freq = 0;

	asprintf(&path, "%s/%s", dev.dtnode, "qcom,gpu-pwrlevels");

	ret = nftw(path, find_freqs_fn, 64, 0);
	if (ret < 0)
		err(1, "could not find power levels");

	free(path);
}

static int
find_device_fn(const char *fpath, const struct stat *sb, int typeflag, struct FTW *ftwbuf)
{
	const char *fname = fpath + ftwbuf->base;
	int sz;

	if (strcmp(fname, "compatible") == 0) {
		char *str = readfile(fpath, &sz);
		if ((strcmp(str, "qcom,adreno-3xx") == 0) ||
				(strcmp(str, "qcom,kgsl-3d0") == 0) ||
				(strstr(str, "qcom,adreno") == str)) {
			int dlen = strlen(fpath) - strlen("/compatible");
			dev.dtnode = malloc(dlen + 1);
			memcpy(dev.dtnode, fpath, dlen);
			printf("found dt node: %s\n", dev.dtnode);
		}
		free(str);
	}
	if (dev.dtnode) {
		/* we found it! */
		return 1;
	}
	return 0;
}

static void
find_device(void)
{
	int ret, fd;
	uint32_t *buf;

	ret = nftw("/proc/device-tree/", find_device_fn, 64, 0);
	if (ret < 0)
		err(1, "could not find adreno gpu");

	if (!dev.dtnode)
		errx(1, "could not find qcom,adreno-3xx node");

	fd = open("/dev/dri/card0", O_RDWR);
	if (fd < 0)
		err(1, "could not open drm device");

	dev.dev  = fd_device_new(fd);
	dev.pipe = fd_pipe_new(dev.dev, FD_PIPE_3D);
	dev.ring = fd_ringbuffer_new(dev.pipe, 0x1000);

	uint64_t val;
	ret = fd_pipe_get_param(dev.pipe, FD_CHIP_ID, &val);
	if (ret) {
		err(1, "could not get gpu-id");
	}
	dev.chipid = val;

#define CHIP_FMT "d%d%d.%d"
#define CHIP_ARGS(chipid) \
		((chipid) >> 24) & 0xff, \
		((chipid) >> 16) & 0xff, \
		((chipid) >> 8) & 0xff, \
		((chipid) >> 0) & 0xff
	printf("device: a%"CHIP_FMT"\n", CHIP_ARGS(dev.chipid));

	buf = readdt("reg");

	/* TODO this is probably insufficent if there are multiple 'reg' nodes..
	 * also, this is assuming the address is 32b (not sure if that will be
	 * always true, although it is with (for example) apq8016..
	 */
	dev.base = ntohl(buf[0]);
	dev.size = ntohl(buf[1]);
	free(buf);

	printf("i/o region at %08lx (size: %x)\n", dev.base, dev.size);

	find_freqs();

	printf("min_freq=%u, max_freq=%u\n", dev.min_freq, dev.max_freq);

	fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0)
		err(1, "could not open /dev/mem");

	dev.io = mmap(0, dev.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, dev.base);
	if (!dev.io)
		err(1, "could not map device");
}

/*
 * perf-monitor
 */

static void
select_counter(struct counter_group *group, int ctr, int n)
{
	group->label[ctr] = enumname(group->countable, n);
	group->reg[ctr].select_val = n;

	/* bashing select register directly while gpu is active will end
	 * in tears.. so we need to write it via the ring:
	 *
	 * TODO it would help startup time, if gpu is loaded, to batch
	 * all the initial writes and do a single flush.. although that
	 * makes things more complicated for capturing inital sample value
	 */
	struct fd_ringbuffer *ring = dev.ring;
	switch (dev.chipid >> 24) {
	case 3:
	case 4:
		OUT_PKT3(ring, CP_WAIT_FOR_IDLE, 1);
		OUT_RING(ring, 0x00000000);

		if (group->reg[ctr].enable_off) {
			OUT_PKT0(ring, group->reg[ctr].enable_off, 1);
			OUT_RING(ring, 0);
		}

		if (group->reg[ctr].clear_off) {
			OUT_PKT0(ring, group->reg[ctr].clear_off, 1);
			OUT_RING(ring, 1);

			OUT_PKT0(ring, group->reg[ctr].clear_off, 1);
			OUT_RING(ring, 0);
		}

		OUT_PKT0(ring, group->reg[ctr].select_off, 1);
		OUT_RING(ring, n);

		if (group->reg[ctr].enable_off) {
			OUT_PKT0(ring, group->reg[ctr].enable_off, 1);
			OUT_RING(ring, 1);
		}

		break;
	case 5:
		OUT_PKT7(ring, CP_WAIT_FOR_IDLE, 0);

		if (group->reg[ctr].enable_off) {
			OUT_PKT4(ring, group->reg[ctr].enable_off, 1);
			OUT_RING(ring, 0);
		}

		if (group->reg[ctr].clear_off) {
			OUT_PKT4(ring, group->reg[ctr].clear_off, 1);
			OUT_RING(ring, 1);

			OUT_PKT4(ring, group->reg[ctr].clear_off, 1);
			OUT_RING(ring, 0);
		}

		OUT_PKT4(ring, group->reg[ctr].select_off, 1);
		OUT_RING(ring, n);

		if (group->reg[ctr].enable_off) {
			OUT_PKT4(ring, group->reg[ctr].enable_off, 1);
			OUT_RING(ring, 1);
		}

		break;
	}

	fd_ringbuffer_flush(ring);

	uint32_t fence = fd_ringbuffer_timestamp(ring);
	fd_pipe_wait(dev.pipe, fence);
	fd_ringbuffer_reset(ring);

	group->last[ctr] = *group->reg[ctr].val_lo;
	group->stime[ctr] = gettime_us();
}

static void
resample_counter(struct counter_group *group, int ctr)
{
	uint32_t val = *group->reg[ctr].val_lo;
	uint32_t t = gettime_us();
	uint32_t dt = delta(group->stime[ctr], t);
	uint32_t dval = delta(group->last[ctr], val);
	group->current[ctr] = (float)dval * 1000000.0 / (float)dt;
	group->last[ctr] = val;
	group->stime[ctr] = t;
}

#define REFRESH_MS 500

/* sample all the counters: */
static void
resample(void)
{
	static uint64_t last_time;
	uint64_t current_time = gettime_us();

	if ((current_time - last_time) < (REFRESH_MS * 1000 / 2))
		return;

	last_time = current_time;

	for (unsigned i = 0; i < dev.ngroups; i++) {
		struct counter_group *group = &dev.groups[i];
		for (unsigned j = 0; j < group->ncounters; j++) {
			resample_counter(group, j);
		}
	}
}

/*
 * The UI
 */

#define COLOR_GROUP_HEADER 1
#define COLOR_FOOTER       2
#define COLOR_INVERSE      3

static int w, h;
static int ctr_width;
static int max_rows, current_cntr = 1;

static void
redraw_footer(WINDOW *win)
{
	char *footer;
	int n;

	n = asprintf(&footer, " fdperf: a%"CHIP_FMT" (%.2fMHz..%.2fMHz)",
			CHIP_ARGS(dev.chipid),
			((float)dev.min_freq) / 1000000.0,
			((float)dev.max_freq) / 1000000.0);

	wmove(win, h - 1, 0);
	wattron(win, COLOR_PAIR(COLOR_FOOTER));
	waddstr(win, footer);
	whline(win, ' ', w - n);
	wattroff(win, COLOR_PAIR(COLOR_FOOTER));

	free(footer);
}

static void
redraw_group_header(WINDOW *win, int row, const char *name)
{
	wmove(win, row, 0);
	wattron(win, A_BOLD);
	wattron(win, COLOR_PAIR(COLOR_GROUP_HEADER));
	waddstr(win, name);
	whline(win, ' ', w - strlen(name));
	wattroff(win, COLOR_PAIR(COLOR_GROUP_HEADER));
	wattroff(win, A_BOLD);
}

static void
redraw_counter_label(WINDOW *win, int row, const char *name, bool selected)
{
	int n = strlen(name);
	assert(n <= ctr_width);
	wmove(win, row, 0);
	whline(win, ' ', ctr_width - n);
	wmove(win, row, ctr_width - n);
	if (selected)
		wattron(win, COLOR_PAIR(COLOR_INVERSE));
	waddstr(win, name);
	if (selected)
		wattroff(win, COLOR_PAIR(COLOR_INVERSE));
	waddstr(win, ": ");
}

static void
redraw_counter_value_cycles(WINDOW *win, float val)
{
	char *str;
	int x = getcurx(win);
	int valwidth = w - x;
	int barwidth, n;

	/* convert to fraction of max freq: */
	val = val / (float)dev.max_freq;

	/* figure out percentage-bar width: */
	barwidth = (int)(val * valwidth);

	n = asprintf(&str, "%.2f%%", 100.0 * val);
	wattron(win, COLOR_PAIR(COLOR_INVERSE));
	waddnstr(win, str, barwidth);
	if (barwidth > n) {
		whline(win, ' ', barwidth - n);
		wmove(win, getcury(win), x + barwidth);
	}
	wattroff(win, COLOR_PAIR(COLOR_INVERSE));
	if (barwidth < n)
		waddstr(win, str + barwidth);
	whline(win, ' ', w - getcurx(win));

	free(str);
}

static void
redraw_counter_value_raw(WINDOW *win, float val)
{
	char *str;
	asprintf(&str, "%.2f", val);
	waddstr(win, str);
	whline(win, ' ', w - getcurx(win));
	free(str);
}

static void
redraw_counter(WINDOW *win, int row, struct counter_group *group,
		int ctr, bool selected)
{
	redraw_counter_label(win, row, group->label[ctr], selected);

	/* quick hack, if the label has "CYCLE" in the name, it is
	 * probably a cycle counter ;-)
	 * Perhaps add more info in rnndb schema to know how to
	 * treat individual counters (ie. which are cycles, and
	 * for those we want to present as a percentage do we
	 * need to scale the result.. ie. is it running at some
	 * multiple or divisor of core clk, etc)
	 *
	 * TODO it would be much more clever to get this from xml
	 * Also.. in some cases I think we want to know how many
	 * units the counter is counting for, ie. if a320 has 2x
	 * shader as a306 we might need to scale the result..
	 */
	if (strstr(group->label[ctr], "CYCLE") ||
			strstr(group->label[ctr], "BUSY") ||
			strstr(group->label[ctr], "IDLE"))
		redraw_counter_value_cycles(win, group->current[ctr]);
	else
		redraw_counter_value_raw(win, group->current[ctr]);
}

static void
redraw(WINDOW *win)
{
	static int scroll = 0;
	int max, row = 0;

	w = getmaxx(win);
	h = getmaxy(win);

	max = h - 3;

	if ((current_cntr - scroll) > (max - 1)) {
		scroll = current_cntr - (max - 1);
	} else if ((current_cntr - 1) < scroll) {
		scroll = current_cntr - 1;
	}

	for (unsigned i = 0; i < dev.ngroups; i++) {
		struct counter_group *group = &dev.groups[i];
		unsigned j = 0;

		/* NOTE skip CP the first CP counter */
		if (i == 0)
			j++;

		if (j < group->ncounters) {
			if ((scroll <= row) && ((row - scroll) < max))
				redraw_group_header(win, row - scroll, group->name);
			row++;
		}

		for (; j < group->ncounters; j++) {
			if ((scroll <= row) && ((row - scroll) < max))
				redraw_counter(win, row - scroll, group, j, row == current_cntr);
			row++;
		}
	}

	/* convert back to physical (unscrolled) offset: */
	row = max;

	redraw_group_header(win, row, "Status");
	row++;

	/* Draw GPU freq row: */
	redraw_counter_label(win, row, "Freq (MHz)", false);
	redraw_counter_value_raw(win, dev.groups[0].current[0] / 1000000.0);
	row++;

	redraw_footer(win);

	refresh();
}

static struct counter_group *
current_counter(int *ctr)
{
	int n = 0;

	for (unsigned i = 0; i < dev.ngroups; i++) {
		struct counter_group *group = &dev.groups[i];
		unsigned j = 0;

		/* NOTE skip CP the first CP counter */
		if (i == 0)
			j++;

		/* account for group header: */
		if (j < group->ncounters) {
			/* cannot select group header.. return null to indicate this
			 * main_ui():
			 */
			if (n == current_cntr)
				return NULL;
			n++;
		}


		for (; j < group->ncounters; j++) {
			if (n == current_cntr) {
				if (ctr)
					*ctr = j;
				return group;
			}
			n++;
		}
	}

	assert(0);
	return NULL;
}

static void
counter_dialog(void)
{
	WINDOW *dialog;
	struct counter_group *group;
	struct rnnenum *en;
	int cnt, current = 0, scroll;

	/* figure out dialog size: */
	int dh = h/2;
	int dw = ctr_width + 2;

	group = current_counter(&cnt);
	en = rnn_findenum(dev.ctx->db, group->countable);

	/* find currently selected idx (note there can be discontinuities
	 * so the selected value does not map 1:1 to current idx)
	 */
	uint32_t selected = group->reg[cnt].select_val;
	for (int i = 0; i < en->valsnum; i++) {
		if (en->vals[i]->value == selected) {
			current = i;
			break;
		}
	}

	/* scrolling offset, if dialog is too small for all the choices: */
	scroll = 0;

	dialog = newwin(dh, dw, (h-dh)/2, (w-dw)/2);
	box(dialog, 0, 0);
	wrefresh(dialog);
	keypad(dialog, TRUE);

	while (true) {
		int max = MIN2(dh - 2, en->valsnum);
		struct rnnvalue *val = NULL;

		if ((current - scroll) >= (dh - 3)) {
			scroll = current - (dh - 3);
		} else if (current < scroll) {
			scroll = current;
		}

		for (int i = 0; i < max; i++) {
			int n = scroll + i;
			wmove(dialog, i+1, 1);
			if (n == current) {
				assert (n < en->valsnum);
				val = en->vals[n];
				wattron(dialog, COLOR_PAIR(COLOR_INVERSE));
			}
			if (n < en->valsnum)
				waddstr(dialog, en->vals[n]->name);
			whline(dialog, ' ', dw - getcurx(dialog) - 1);
			if (n == current)
				wattroff(dialog, COLOR_PAIR(COLOR_INVERSE));
		}

		assert (val);

		switch (wgetch(dialog)) {
		case KEY_UP:
			current = MAX2(0, current - 1);
			break;
		case KEY_DOWN:
			current = MIN2(en->valsnum - 1, current + 1);
			break;
		case KEY_LEFT:
		case KEY_ENTER:
			/* select new sampler */
			select_counter(group, cnt, val->value);
			config_save();
			goto out;
		case 'q':
			goto out;
		default:
			/* ignore */
			break;
		}

		resample();
	}

out:
	wborder(dialog, ' ', ' ', ' ',' ',' ',' ',' ',' ');
	delwin(dialog);
}

static void
scroll_cntr(int amount)
{
	if (amount < 0) {
		current_cntr = MAX2(1, current_cntr + amount);
		if (current_counter(NULL) == NULL) {
			current_cntr = MAX2(1, current_cntr - 1);
		}
	} else {
		current_cntr = MIN2(max_rows - 1, current_cntr + amount);
		if (current_counter(NULL) == NULL)
			current_cntr = MIN2(max_rows - 1, current_cntr + 1);
	}
}

static void
main_ui(void)
{
	WINDOW *mainwin;

	/* curses setup: */
	mainwin = initscr();
	if (!mainwin)
		goto out;

	cbreak();
	wtimeout(mainwin, REFRESH_MS);
	noecho();
	keypad(mainwin, TRUE);
	curs_set(0);
	start_color();
	init_pair(COLOR_GROUP_HEADER, COLOR_WHITE, COLOR_GREEN);
	init_pair(COLOR_FOOTER,       COLOR_WHITE, COLOR_BLUE);
	init_pair(COLOR_INVERSE,      COLOR_BLACK, COLOR_WHITE);

	while (true) {
		switch (wgetch(mainwin)) {
		case KEY_UP:
			scroll_cntr(-1);
			break;
		case KEY_DOWN:
			scroll_cntr(+1);
			break;
		case KEY_NPAGE:  /* page-down */
			/* TODO figure out # of rows visible? */
			scroll_cntr(+15);
			break;
		case KEY_PPAGE:  /* page-up */
			/* TODO figure out # of rows visible? */
			scroll_cntr(-15);
			break;
		case KEY_RIGHT:
			counter_dialog();
			break;
		case 'q':
			goto out;
			break;
		default:
			/* ignore */
			break;
		}
		resample();
		redraw(mainwin);
	}

	/* restore settings.. maybe we need an atexit()??*/
out:
	delwin(mainwin);
	endwin();
	refresh();
}

static void
setup_counter_groups(void)
{
	for (unsigned i = 0; i < dev.ngroups; i++) {
		struct counter_group *group = &dev.groups[i];
		max_rows += group->ncounters + 1;

		/* the first CP counter is hidden: */
		if (i == 0) {
			max_rows--;
			if (group->ncounters <= 1)
				max_rows--;
		}

		for (unsigned j = 0; j < group->ncounters; j++) {
			group->reg[j].select_off = regelem(dev.dom, group->counter[j].select)->offset;
			if (group->counter[j].enable)
				group->reg[j].enable_off = regelem(dev.dom, group->counter[j].enable)->offset;
			if (group->counter[j].clear)
				group->reg[j].clear_off = regelem(dev.dom, group->counter[j].clear)->offset;
			group->reg[j].val_hi = dev.io + (regelem(dev.dom, group->counter[j].val_hi)->offset * 4);
			group->reg[j].val_lo = dev.io + (regelem(dev.dom, group->counter[j].val_lo)->offset * 4);
			select_counter(group, j, j);

			ctr_width = MAX2(ctr_width, strlen(enumname(group->countable, j)) + 1);
		}
	}
}

/*
 * configuration / persistence
 */

static config_t cfg;
static config_setting_t *setting;

static void
config_save(void)
{
	for (unsigned i = 0; i < dev.ngroups; i++) {
		struct counter_group *group = &dev.groups[i];
		unsigned j = 0;

		/* NOTE skip CP the first CP counter */
		if (i == 0)
			j++;

		for (; j < group->ncounters; j++) {
			config_setting_t *s =
				config_setting_lookup(setting, group->counter[j].select);
			config_setting_set_int(s, group->reg[j].select_val);
		}
	}

	config_write_file(&cfg, "fdperf.cfg");
}

static void
config_restore(void)
{
	char *str;

	config_init(&cfg);

	/* Read the file. If there is an error, report it and exit. */
	if(!config_read_file(&cfg, "fdperf.cfg")) {
		warn("could not restore settings");
	}

	config_setting_t *root = config_root_setting(&cfg);

	/* per device settings: */
	asprintf(&str, "a%dxx", dev.chipid >> 24);
	setting = config_setting_get_member(root, str);
	if (!setting)
		setting = config_setting_add(root, str, CONFIG_TYPE_GROUP);
	free(str);

	for (unsigned i = 0; i < dev.ngroups; i++) {
		struct counter_group *group = &dev.groups[i];
		unsigned j = 0;

		/* NOTE skip CP the first CP counter */
		if (i == 0)
			j++;

		for (; j < group->ncounters; j++) {
			config_setting_t *s =
				config_setting_lookup(setting, group->counter[j].select);
			if (!s) {
				config_setting_add(setting, group->counter[j].select,
						CONFIG_TYPE_INT);
				continue;
			}
			select_counter(group, j, config_setting_get_int(s));
		}
	}
}

/*
 * per-generation table of counters:
 */

static struct counter_group a3xx_counters[] = {
	{ "CP", "a3xx_cp_perfcounter_select", 1, {
		{ "CP_PERFCOUNTER_SELECT", "RBBM_PERFCTR_CP_0_HI", "RBBM_PERFCTR_CP_0_LO" },
	}},
	{ "GRAS TSE", "a3xx_gras_tse_perfcounter_select", 2, {
		{ "GRAS_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_TSE_0_HI", "RBBM_PERFCTR_TSE_0_LO" },
		{ "GRAS_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_TSE_1_HI", "RBBM_PERFCTR_TSE_1_LO" },
	}},
	{ "GRAS RAS", "a3xx_gras_ras_perfcounter_select", 2, {
		{ "GRAS_PERFCOUNTER2_SELECT", "RBBM_PERFCTR_RAS_0_HI", "RBBM_PERFCTR_RAS_0_LO" },
		{ "GRAS_PERFCOUNTER3_SELECT", "RBBM_PERFCTR_RAS_1_HI", "RBBM_PERFCTR_RAS_1_LO" },
	}},
	{ "HLSQ", "a3xx_hlsq_perfcounter_select", 6, {
		{ "HLSQ_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_HLSQ_0_HI", "RBBM_PERFCTR_HLSQ_0_LO" },
		{ "HLSQ_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_HLSQ_1_HI", "RBBM_PERFCTR_HLSQ_1_LO" },
		{ "HLSQ_PERFCOUNTER2_SELECT", "RBBM_PERFCTR_HLSQ_2_HI", "RBBM_PERFCTR_HLSQ_2_LO" },
		{ "HLSQ_PERFCOUNTER3_SELECT", "RBBM_PERFCTR_HLSQ_3_HI", "RBBM_PERFCTR_HLSQ_3_LO" },
		{ "HLSQ_PERFCOUNTER4_SELECT", "RBBM_PERFCTR_HLSQ_4_HI", "RBBM_PERFCTR_HLSQ_4_LO" },
		{ "HLSQ_PERFCOUNTER5_SELECT", "RBBM_PERFCTR_HLSQ_5_HI", "RBBM_PERFCTR_HLSQ_5_LO" },
	}},
	{ "PC", "a3xx_pc_perfcounter_select", 4, {
		{ "PC_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_PC_0_HI", "RBBM_PERFCTR_PC_0_LO" },
		{ "PC_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_PC_1_HI", "RBBM_PERFCTR_PC_1_LO" },
		{ "PC_PERFCOUNTER2_SELECT", "RBBM_PERFCTR_PC_2_HI", "RBBM_PERFCTR_PC_2_LO" },
		{ "PC_PERFCOUNTER3_SELECT", "RBBM_PERFCTR_PC_3_HI", "RBBM_PERFCTR_PC_3_LO" },
	}},
	{ "RB", "a3xx_rb_perfcounter_select", 2, {
		{ "RB_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_RB_0_HI", "RBBM_PERFCTR_RB_0_LO" },
		{ "RB_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_RB_1_HI", "RBBM_PERFCTR_RB_1_LO" },
	}},
	{ "RBBM", "a3xx_rbbm_perfcounter_select", 2, {
		{ "RBBM_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_RBBM_0_HI", "RBBM_PERFCTR_RBBM_0_LO" },
		{ "RBBM_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_RBBM_1_HI", "RBBM_PERFCTR_RBBM_1_LO" },
	}},
	{ "SP", "a3xx_sp_perfcounter_select", 8, {
		{ "SP_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_SP_0_HI", "RBBM_PERFCTR_SP_0_LO" },
		{ "SP_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_SP_1_HI", "RBBM_PERFCTR_SP_1_LO" },
		{ "SP_PERFCOUNTER2_SELECT", "RBBM_PERFCTR_SP_2_HI", "RBBM_PERFCTR_SP_2_LO" },
		{ "SP_PERFCOUNTER3_SELECT", "RBBM_PERFCTR_SP_3_HI", "RBBM_PERFCTR_SP_3_LO" },
		{ "SP_PERFCOUNTER4_SELECT", "RBBM_PERFCTR_SP_4_HI", "RBBM_PERFCTR_SP_4_LO" },
		{ "SP_PERFCOUNTER5_SELECT", "RBBM_PERFCTR_SP_5_HI", "RBBM_PERFCTR_SP_5_LO" },
		{ "SP_PERFCOUNTER6_SELECT", "RBBM_PERFCTR_SP_6_HI", "RBBM_PERFCTR_SP_6_LO" },
		{ "SP_PERFCOUNTER7_SELECT", "RBBM_PERFCTR_SP_7_HI", "RBBM_PERFCTR_SP_7_LO" },
	}},
	{ "TP", "a3xx_tp_perfcounter_select", 6, {
		{ "TP_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_TP_0_HI", "RBBM_PERFCTR_TP_0_LO" },
		{ "TP_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_TP_1_HI", "RBBM_PERFCTR_TP_1_LO" },
		{ "TP_PERFCOUNTER2_SELECT", "RBBM_PERFCTR_TP_2_HI", "RBBM_PERFCTR_TP_2_LO" },
		{ "TP_PERFCOUNTER3_SELECT", "RBBM_PERFCTR_TP_3_HI", "RBBM_PERFCTR_TP_3_LO" },
		{ "TP_PERFCOUNTER4_SELECT", "RBBM_PERFCTR_TP_4_HI", "RBBM_PERFCTR_TP_4_LO" },
		{ "TP_PERFCOUNTER5_SELECT", "RBBM_PERFCTR_TP_5_HI", "RBBM_PERFCTR_TP_5_LO" },
	}},
	{ "VFD", "a3xx_vfd_perfcounter_select", 2, {
		{ "VFD_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_VFD_0_HI", "RBBM_PERFCTR_VFD_0_LO" },
		{ "VFD_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_VFD_1_HI", "RBBM_PERFCTR_VFD_1_LO" },
	}},
	{ "VPC", "a3xx_vpc_perfcounter_select", 2, {
		{ "VPC_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_VPC_0_HI", "RBBM_PERFCTR_VPC_0_LO" },
		{ "VPC_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_VPC_1_HI", "RBBM_PERFCTR_VPC_1_LO" },
	}},
	{ "UCHE", "a3xx_uche_perfcounter_select", 6, {
		{ "UCHE_PERFCOUNTER0_SELECT", "RBBM_PERFCTR_UCHE_0_HI", "RBBM_PERFCTR_UCHE_0_LO" },
		{ "UCHE_PERFCOUNTER1_SELECT", "RBBM_PERFCTR_UCHE_1_HI", "RBBM_PERFCTR_UCHE_1_LO" },
		{ "UCHE_PERFCOUNTER2_SELECT", "RBBM_PERFCTR_UCHE_2_HI", "RBBM_PERFCTR_UCHE_2_LO" },
		{ "UCHE_PERFCOUNTER3_SELECT", "RBBM_PERFCTR_UCHE_3_HI", "RBBM_PERFCTR_UCHE_3_LO" },
		{ "UCHE_PERFCOUNTER4_SELECT", "RBBM_PERFCTR_UCHE_4_HI", "RBBM_PERFCTR_UCHE_4_LO" },
		{ "UCHE_PERFCOUNTER5_SELECT", "RBBM_PERFCTR_UCHE_5_HI", "RBBM_PERFCTR_UCHE_5_LO" },
	}},
};

static struct counter_group a4xx_counters[] = {
	{ "CP", "a4xx_cp_perfcounter_select", 2, {
		{ "CP_PERFCTR_CP_SEL_0", "RBBM_PERFCTR_CP_0_HI", "RBBM_PERFCTR_CP_0_LO" },
		{ "CP_PERFCTR_CP_SEL_1", "RBBM_PERFCTR_CP_1_HI", "RBBM_PERFCTR_CP_1_LO" },
#if 0
		/* see a420_perfcounters_cp in downstream kernel.. the association of
		 * select and readback regs is messed up.  Just stick our heads in the
		 * sand for now and ignore everything but the first two.
		 */
		{ "CP_PERFCTR_CP_SEL_2", "RBBM_PERFCTR_CP_2_HI", "RBBM_PERFCTR_CP_2_LO" },
		{ "CP_PERFCTR_CP_SEL_3", "RBBM_PERFCTR_CP_3_HI", "RBBM_PERFCTR_CP_3_LO" },
		{ "CP_PERFCTR_CP_SEL_4", "RBBM_PERFCTR_CP_4_HI", "RBBM_PERFCTR_CP_4_LO" },
		{ "CP_PERFCTR_CP_SEL_5", "RBBM_PERFCTR_CP_5_HI", "RBBM_PERFCTR_CP_5_LO" },
		{ "CP_PERFCTR_CP_SEL_6", "RBBM_PERFCTR_CP_6_HI", "RBBM_PERFCTR_CP_6_LO" },
		{ "CP_PERFCTR_CP_SEL_7", "RBBM_PERFCTR_CP_7_HI", "RBBM_PERFCTR_CP_7_LO" },
#endif
	}},
	{ "CCU", "a4xx_ccu_perfcounter_select", 4, {
		{ "RB_PERFCTR_CCU_SEL_0", "RBBM_PERFCTR_CCU_0_HI", "RBBM_PERFCTR_CCU_0_LO" },
		{ "RB_PERFCTR_CCU_SEL_1", "RBBM_PERFCTR_CCU_1_HI", "RBBM_PERFCTR_CCU_1_LO" },
		{ "RB_PERFCTR_CCU_SEL_2", "RBBM_PERFCTR_CCU_2_HI", "RBBM_PERFCTR_CCU_2_LO" },
		{ "RB_PERFCTR_CCU_SEL_3", "RBBM_PERFCTR_CCU_3_HI", "RBBM_PERFCTR_CCU_3_LO" },
	}},
	{ "GRAS RAS", "a4xx_gras_ras_perfcounter_select", 4, {
		{ "GRAS_PERFCTR_RAS_SEL_0", "RBBM_PERFCTR_RAS_0_HI", "RBBM_PERFCTR_RAS_0_LO" },
		{ "GRAS_PERFCTR_RAS_SEL_1", "RBBM_PERFCTR_RAS_1_HI", "RBBM_PERFCTR_RAS_1_LO" },
		{ "GRAS_PERFCTR_RAS_SEL_2", "RBBM_PERFCTR_RAS_2_HI", "RBBM_PERFCTR_RAS_2_LO" },
		{ "GRAS_PERFCTR_RAS_SEL_3", "RBBM_PERFCTR_RAS_3_HI", "RBBM_PERFCTR_RAS_3_LO" },
	}},
	{ "GRAS TSE", "a4xx_gras_tse_perfcounter_select", 4, {
		{ "GRAS_PERFCTR_TSE_SEL_0", "RBBM_PERFCTR_TSE_0_HI", "RBBM_PERFCTR_TSE_0_LO" },
		{ "GRAS_PERFCTR_TSE_SEL_1", "RBBM_PERFCTR_TSE_1_HI", "RBBM_PERFCTR_TSE_1_LO" },
		{ "GRAS_PERFCTR_TSE_SEL_2", "RBBM_PERFCTR_TSE_2_HI", "RBBM_PERFCTR_TSE_2_LO" },
		{ "GRAS_PERFCTR_TSE_SEL_3", "RBBM_PERFCTR_TSE_3_HI", "RBBM_PERFCTR_TSE_3_LO" },
	}},
	{ "HLSQ", "a4xx_hlsq_perfcounter_select", 8, {
		{ "HLSQ_PERFCTR_HLSQ_SEL_0", "RBBM_PERFCTR_HLSQ_0_HI", "RBBM_PERFCTR_HLSQ_0_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_1", "RBBM_PERFCTR_HLSQ_1_HI", "RBBM_PERFCTR_HLSQ_1_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_2", "RBBM_PERFCTR_HLSQ_2_HI", "RBBM_PERFCTR_HLSQ_2_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_3", "RBBM_PERFCTR_HLSQ_3_HI", "RBBM_PERFCTR_HLSQ_3_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_4", "RBBM_PERFCTR_HLSQ_4_HI", "RBBM_PERFCTR_HLSQ_4_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_5", "RBBM_PERFCTR_HLSQ_5_HI", "RBBM_PERFCTR_HLSQ_5_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_6", "RBBM_PERFCTR_HLSQ_6_HI", "RBBM_PERFCTR_HLSQ_6_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_7", "RBBM_PERFCTR_HLSQ_7_HI", "RBBM_PERFCTR_HLSQ_7_LO" },
	}},
	{ "PC", "a4xx_pc_perfcounter_select", 8, {
		{ "PC_PERFCTR_PC_SEL_0", "RBBM_PERFCTR_PC_0_HI", "RBBM_PERFCTR_PC_0_LO" },
		{ "PC_PERFCTR_PC_SEL_1", "RBBM_PERFCTR_PC_1_HI", "RBBM_PERFCTR_PC_1_LO" },
		{ "PC_PERFCTR_PC_SEL_2", "RBBM_PERFCTR_PC_2_HI", "RBBM_PERFCTR_PC_2_LO" },
		{ "PC_PERFCTR_PC_SEL_3", "RBBM_PERFCTR_PC_3_HI", "RBBM_PERFCTR_PC_3_LO" },
		{ "PC_PERFCTR_PC_SEL_4", "RBBM_PERFCTR_PC_4_HI", "RBBM_PERFCTR_PC_4_LO" },
		{ "PC_PERFCTR_PC_SEL_5", "RBBM_PERFCTR_PC_5_HI", "RBBM_PERFCTR_PC_5_LO" },
		{ "PC_PERFCTR_PC_SEL_6", "RBBM_PERFCTR_PC_6_HI", "RBBM_PERFCTR_PC_6_LO" },
		{ "PC_PERFCTR_PC_SEL_7", "RBBM_PERFCTR_PC_7_HI", "RBBM_PERFCTR_PC_7_LO" },
	}},
	{ "RB", "a4xx_rb_perfcounter_select", 8, {
		{ "RB_PERFCTR_RB_SEL_0", "RBBM_PERFCTR_RB_0_HI", "RBBM_PERFCTR_RB_0_LO" },
		{ "RB_PERFCTR_RB_SEL_1", "RBBM_PERFCTR_RB_1_HI", "RBBM_PERFCTR_RB_1_LO" },
		{ "RB_PERFCTR_RB_SEL_2", "RBBM_PERFCTR_RB_2_HI", "RBBM_PERFCTR_RB_2_LO" },
		{ "RB_PERFCTR_RB_SEL_3", "RBBM_PERFCTR_RB_3_HI", "RBBM_PERFCTR_RB_3_LO" },
		{ "RB_PERFCTR_RB_SEL_4", "RBBM_PERFCTR_RB_4_HI", "RBBM_PERFCTR_RB_4_LO" },
		{ "RB_PERFCTR_RB_SEL_5", "RBBM_PERFCTR_RB_5_HI", "RBBM_PERFCTR_RB_5_LO" },
		{ "RB_PERFCTR_RB_SEL_6", "RBBM_PERFCTR_RB_6_HI", "RBBM_PERFCTR_RB_6_LO" },
		{ "RB_PERFCTR_RB_SEL_7", "RBBM_PERFCTR_RB_7_HI", "RBBM_PERFCTR_RB_7_LO" },
	}},
	{ "RBBM", "a4xx_rbbm_perfcounter_select", 4, {
		{ "RBBM_PERFCTR_RBBM_SEL_0", "RBBM_PERFCTR_RBBM_0_HI", "RBBM_PERFCTR_RBBM_0_HI" },
		{ "RBBM_PERFCTR_RBBM_SEL_1", "RBBM_PERFCTR_RBBM_1_HI", "RBBM_PERFCTR_RBBM_1_HI" },
		{ "RBBM_PERFCTR_RBBM_SEL_2", "RBBM_PERFCTR_RBBM_2_HI", "RBBM_PERFCTR_RBBM_2_HI" },
		{ "RBBM_PERFCTR_RBBM_SEL_3", "RBBM_PERFCTR_RBBM_3_HI", "RBBM_PERFCTR_RBBM_3_HI" },
	}},
	/* NOTE.. also RBBM_ALWAYSON_COUNTER_HI/RBBM_ALWAYSON_COUNTER_LO, which has
	 * no select register..
	 */
	{ "SP", "a4xx_sp_perfcounter_select", 12, {
		{ "SP_PERFCTR_SP_SEL_0",  "RBBM_PERFCTR_SP_0_HI",  "RBBM_PERFCTR_SP_0_LO" },
		{ "SP_PERFCTR_SP_SEL_1",  "RBBM_PERFCTR_SP_1_HI",  "RBBM_PERFCTR_SP_1_LO" },
		{ "SP_PERFCTR_SP_SEL_2",  "RBBM_PERFCTR_SP_2_HI",  "RBBM_PERFCTR_SP_2_LO" },
		{ "SP_PERFCTR_SP_SEL_3",  "RBBM_PERFCTR_SP_3_HI",  "RBBM_PERFCTR_SP_3_LO" },
		{ "SP_PERFCTR_SP_SEL_4",  "RBBM_PERFCTR_SP_4_HI",  "RBBM_PERFCTR_SP_4_LO" },
		{ "SP_PERFCTR_SP_SEL_5",  "RBBM_PERFCTR_SP_5_HI",  "RBBM_PERFCTR_SP_5_LO" },
		{ "SP_PERFCTR_SP_SEL_6",  "RBBM_PERFCTR_SP_6_HI",  "RBBM_PERFCTR_SP_6_LO" },
		{ "SP_PERFCTR_SP_SEL_7",  "RBBM_PERFCTR_SP_7_HI",  "RBBM_PERFCTR_SP_7_LO" },
		{ "SP_PERFCTR_SP_SEL_8",  "RBBM_PERFCTR_SP_8_HI",  "RBBM_PERFCTR_SP_8_LO" },
		{ "SP_PERFCTR_SP_SEL_9",  "RBBM_PERFCTR_SP_9_HI",  "RBBM_PERFCTR_SP_9_LO" },
		{ "SP_PERFCTR_SP_SEL_10", "RBBM_PERFCTR_SP_10_HI", "RBBM_PERFCTR_SP_10_LO" },
		{ "SP_PERFCTR_SP_SEL_11", "RBBM_PERFCTR_SP_11_HI", "RBBM_PERFCTR_SP_11_LO" },
	}},
	{ "TP", "a4xx_tp_perfcounter_select", 8, {
		{ "TPL1_PERFCTR_TP_SEL_0", "RBBM_PERFCTR_TP_0_HI", "RBBM_PERFCTR_TP_0_LO" },
		{ "TPL1_PERFCTR_TP_SEL_1", "RBBM_PERFCTR_TP_1_HI", "RBBM_PERFCTR_TP_1_LO" },
		{ "TPL1_PERFCTR_TP_SEL_2", "RBBM_PERFCTR_TP_2_HI", "RBBM_PERFCTR_TP_2_LO" },
		{ "TPL1_PERFCTR_TP_SEL_3", "RBBM_PERFCTR_TP_3_HI", "RBBM_PERFCTR_TP_3_LO" },
		{ "TPL1_PERFCTR_TP_SEL_4", "RBBM_PERFCTR_TP_4_HI", "RBBM_PERFCTR_TP_4_LO" },
		{ "TPL1_PERFCTR_TP_SEL_5", "RBBM_PERFCTR_TP_5_HI", "RBBM_PERFCTR_TP_5_LO" },
		{ "TPL1_PERFCTR_TP_SEL_6", "RBBM_PERFCTR_TP_6_HI", "RBBM_PERFCTR_TP_6_LO" },
		{ "TPL1_PERFCTR_TP_SEL_7", "RBBM_PERFCTR_TP_7_HI", "RBBM_PERFCTR_TP_7_LO" },
	}},
	{ "UCHE", "a4xx_uche_perfcounter_select", 8, {
		{ "UCHE_PERFCTR_UCHE_SEL_0", "RBBM_PERFCTR_UCHE_0_HI", "RBBM_PERFCTR_UCHE_0_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_1", "RBBM_PERFCTR_UCHE_1_HI", "RBBM_PERFCTR_UCHE_1_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_2", "RBBM_PERFCTR_UCHE_2_HI", "RBBM_PERFCTR_UCHE_2_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_3", "RBBM_PERFCTR_UCHE_3_HI", "RBBM_PERFCTR_UCHE_3_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_4", "RBBM_PERFCTR_UCHE_4_HI", "RBBM_PERFCTR_UCHE_4_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_5", "RBBM_PERFCTR_UCHE_5_HI", "RBBM_PERFCTR_UCHE_5_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_6", "RBBM_PERFCTR_UCHE_6_HI", "RBBM_PERFCTR_UCHE_6_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_7", "RBBM_PERFCTR_UCHE_7_HI", "RBBM_PERFCTR_UCHE_7_LO" },
	}},
	{ "VFD", "a4xx_vfd_perfcounter_select", 8, {
		{ "VFD_PERFCTR_VFD_SEL_0", "RBBM_PERFCTR_VFD_0_HI", "RBBM_PERFCTR_VFD_0_LO" },
		{ "VFD_PERFCTR_VFD_SEL_1", "RBBM_PERFCTR_VFD_1_HI", "RBBM_PERFCTR_VFD_1_LO" },
		{ "VFD_PERFCTR_VFD_SEL_2", "RBBM_PERFCTR_VFD_2_HI", "RBBM_PERFCTR_VFD_2_LO" },
		{ "VFD_PERFCTR_VFD_SEL_3", "RBBM_PERFCTR_VFD_3_HI", "RBBM_PERFCTR_VFD_3_LO" },
		{ "VFD_PERFCTR_VFD_SEL_4", "RBBM_PERFCTR_VFD_4_HI", "RBBM_PERFCTR_VFD_4_LO" },
		{ "VFD_PERFCTR_VFD_SEL_5", "RBBM_PERFCTR_VFD_5_HI", "RBBM_PERFCTR_VFD_5_LO" },
		{ "VFD_PERFCTR_VFD_SEL_6", "RBBM_PERFCTR_VFD_6_HI", "RBBM_PERFCTR_VFD_6_LO" },
		{ "VFD_PERFCTR_VFD_SEL_7", "RBBM_PERFCTR_VFD_7_HI", "RBBM_PERFCTR_VFD_7_LO" },
	}},
	{ "VPC", "a4xx_vpc_perfcounter_select", 4, {
		{ "VPC_PERFCTR_VPC_SEL_0", "RBBM_PERFCTR_VPC_0_HI", "RBBM_PERFCTR_VPC_0_LO" },
		{ "VPC_PERFCTR_VPC_SEL_1", "RBBM_PERFCTR_VPC_1_HI", "RBBM_PERFCTR_VPC_1_LO" },
		{ "VPC_PERFCTR_VPC_SEL_2", "RBBM_PERFCTR_VPC_2_HI", "RBBM_PERFCTR_VPC_2_LO" },
		{ "VPC_PERFCTR_VPC_SEL_3", "RBBM_PERFCTR_VPC_3_HI", "RBBM_PERFCTR_VPC_3_LO" },
	}},
	{ "VSC", "a4xx_vsc_perfcounter_select", 2, {
		{ "VSC_PERFCTR_VSC_SEL_0", "RBBM_PERFCTR_VSC_0_HI", "RBBM_PERFCTR_VSC_0_LO" },
		{ "VSC_PERFCTR_VSC_SEL_1", "RBBM_PERFCTR_VSC_1_HI", "RBBM_PERFCTR_VSC_1_LO" },
	}},
	{ "VBIF", "a4xx_vbif_perfcounter_select", 4, {
		{ "VBIF_PERF_CNT_SEL0", "VBIF_PERF_CNT_LOW0", "VBIF_PERF_CNT_HIGH0", "VBIF_PERF_CNT_EN0" },
		{ "VBIF_PERF_CNT_SEL1", "VBIF_PERF_CNT_LOW1", "VBIF_PERF_CNT_HIGH1", "VBIF_PERF_CNT_EN1" },
		{ "VBIF_PERF_CNT_SEL2", "VBIF_PERF_CNT_LOW2", "VBIF_PERF_CNT_HIGH2", "VBIF_PERF_CNT_EN2" },
		{ "VBIF_PERF_CNT_SEL3", "VBIF_PERF_CNT_LOW3", "VBIF_PERF_CNT_HIGH3", "VBIF_PERF_CNT_EN3" },
	}},
};

static struct counter_group a5xx_counters[] = {
	{ "CP", "a5xx_cp_perfcounter_select", 8, {
		{ "CP_PERFCTR_CP_SEL_0", "RBBM_PERFCTR_CP_0_HI", "RBBM_PERFCTR_CP_0_LO" },
		{ "CP_PERFCTR_CP_SEL_1", "RBBM_PERFCTR_CP_1_HI", "RBBM_PERFCTR_CP_1_LO" },
		{ "CP_PERFCTR_CP_SEL_2", "RBBM_PERFCTR_CP_2_HI", "RBBM_PERFCTR_CP_2_LO" },
		{ "CP_PERFCTR_CP_SEL_3", "RBBM_PERFCTR_CP_3_HI", "RBBM_PERFCTR_CP_3_LO" },
		{ "CP_PERFCTR_CP_SEL_4", "RBBM_PERFCTR_CP_4_HI", "RBBM_PERFCTR_CP_4_LO" },
		{ "CP_PERFCTR_CP_SEL_5", "RBBM_PERFCTR_CP_5_HI", "RBBM_PERFCTR_CP_5_LO" },
		{ "CP_PERFCTR_CP_SEL_6", "RBBM_PERFCTR_CP_6_HI", "RBBM_PERFCTR_CP_6_LO" },
		{ "CP_PERFCTR_CP_SEL_7", "RBBM_PERFCTR_CP_7_HI", "RBBM_PERFCTR_CP_7_LO" },
	}},
	{ "CCU", "a5xx_ccu_perfcounter_select", 4, {
		{ "RB_PERFCTR_CCU_SEL_0", "RBBM_PERFCTR_CCU_0_HI", "RBBM_PERFCTR_CCU_0_LO" },
		{ "RB_PERFCTR_CCU_SEL_1", "RBBM_PERFCTR_CCU_1_HI", "RBBM_PERFCTR_CCU_1_LO" },
		{ "RB_PERFCTR_CCU_SEL_2", "RBBM_PERFCTR_CCU_2_HI", "RBBM_PERFCTR_CCU_2_LO" },
		{ "RB_PERFCTR_CCU_SEL_3", "RBBM_PERFCTR_CCU_3_HI", "RBBM_PERFCTR_CCU_3_LO" },
	}},
	{ "GRAS RAS", "a5xx_ras_perfcounter_select", 4, {
		{ "GRAS_PERFCTR_RAS_SEL_0", "RBBM_PERFCTR_RAS_0_HI", "RBBM_PERFCTR_RAS_0_LO" },
		{ "GRAS_PERFCTR_RAS_SEL_1", "RBBM_PERFCTR_RAS_1_HI", "RBBM_PERFCTR_RAS_1_LO" },
		{ "GRAS_PERFCTR_RAS_SEL_2", "RBBM_PERFCTR_RAS_2_HI", "RBBM_PERFCTR_RAS_2_LO" },
		{ "GRAS_PERFCTR_RAS_SEL_3", "RBBM_PERFCTR_RAS_3_HI", "RBBM_PERFCTR_RAS_3_LO" },
	}},
	{ "GRAS TSE", "a5xx_tse_perfcounter_select", 4, {
		{ "GRAS_PERFCTR_TSE_SEL_0", "RBBM_PERFCTR_TSE_0_HI", "RBBM_PERFCTR_TSE_0_LO" },
		{ "GRAS_PERFCTR_TSE_SEL_1", "RBBM_PERFCTR_TSE_1_HI", "RBBM_PERFCTR_TSE_1_LO" },
		{ "GRAS_PERFCTR_TSE_SEL_2", "RBBM_PERFCTR_TSE_2_HI", "RBBM_PERFCTR_TSE_2_LO" },
		{ "GRAS_PERFCTR_TSE_SEL_3", "RBBM_PERFCTR_TSE_3_HI", "RBBM_PERFCTR_TSE_3_LO" },
	}},
	{ "HLSQ", "a5xx_hlsq_perfcounter_select", 8, {
		{ "HLSQ_PERFCTR_HLSQ_SEL_0", "RBBM_PERFCTR_HLSQ_0_HI", "RBBM_PERFCTR_HLSQ_0_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_1", "RBBM_PERFCTR_HLSQ_1_HI", "RBBM_PERFCTR_HLSQ_1_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_2", "RBBM_PERFCTR_HLSQ_2_HI", "RBBM_PERFCTR_HLSQ_2_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_3", "RBBM_PERFCTR_HLSQ_3_HI", "RBBM_PERFCTR_HLSQ_3_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_4", "RBBM_PERFCTR_HLSQ_4_HI", "RBBM_PERFCTR_HLSQ_4_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_5", "RBBM_PERFCTR_HLSQ_5_HI", "RBBM_PERFCTR_HLSQ_5_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_6", "RBBM_PERFCTR_HLSQ_6_HI", "RBBM_PERFCTR_HLSQ_6_LO" },
		{ "HLSQ_PERFCTR_HLSQ_SEL_7", "RBBM_PERFCTR_HLSQ_7_HI", "RBBM_PERFCTR_HLSQ_7_LO" },
	}},
	{ "PC", "a5xx_pc_perfcounter_select", 8, {
		{ "PC_PERFCTR_PC_SEL_0", "RBBM_PERFCTR_PC_0_HI", "RBBM_PERFCTR_PC_0_LO" },
		{ "PC_PERFCTR_PC_SEL_1", "RBBM_PERFCTR_PC_1_HI", "RBBM_PERFCTR_PC_1_LO" },
		{ "PC_PERFCTR_PC_SEL_2", "RBBM_PERFCTR_PC_2_HI", "RBBM_PERFCTR_PC_2_LO" },
		{ "PC_PERFCTR_PC_SEL_3", "RBBM_PERFCTR_PC_3_HI", "RBBM_PERFCTR_PC_3_LO" },
		{ "PC_PERFCTR_PC_SEL_4", "RBBM_PERFCTR_PC_4_HI", "RBBM_PERFCTR_PC_4_LO" },
		{ "PC_PERFCTR_PC_SEL_5", "RBBM_PERFCTR_PC_5_HI", "RBBM_PERFCTR_PC_5_LO" },
		{ "PC_PERFCTR_PC_SEL_6", "RBBM_PERFCTR_PC_6_HI", "RBBM_PERFCTR_PC_6_LO" },
		{ "PC_PERFCTR_PC_SEL_7", "RBBM_PERFCTR_PC_7_HI", "RBBM_PERFCTR_PC_7_LO" },
	}},
	{ "RB", "a5xx_rb_perfcounter_select", 8, {
		{ "RB_PERFCTR_RB_SEL_0", "RBBM_PERFCTR_RB_0_HI", "RBBM_PERFCTR_RB_0_LO" },
		{ "RB_PERFCTR_RB_SEL_1", "RBBM_PERFCTR_RB_1_HI", "RBBM_PERFCTR_RB_1_LO" },
		{ "RB_PERFCTR_RB_SEL_2", "RBBM_PERFCTR_RB_2_HI", "RBBM_PERFCTR_RB_2_LO" },
		{ "RB_PERFCTR_RB_SEL_3", "RBBM_PERFCTR_RB_3_HI", "RBBM_PERFCTR_RB_3_LO" },
		{ "RB_PERFCTR_RB_SEL_4", "RBBM_PERFCTR_RB_4_HI", "RBBM_PERFCTR_RB_4_LO" },
		{ "RB_PERFCTR_RB_SEL_5", "RBBM_PERFCTR_RB_5_HI", "RBBM_PERFCTR_RB_5_LO" },
		{ "RB_PERFCTR_RB_SEL_6", "RBBM_PERFCTR_RB_6_HI", "RBBM_PERFCTR_RB_6_LO" },
		{ "RB_PERFCTR_RB_SEL_7", "RBBM_PERFCTR_RB_7_HI", "RBBM_PERFCTR_RB_7_LO" },
	}},
	{ "RBBM", "a5xx_rbbm_perfcounter_select", 4, {
		{ "RBBM_PERFCTR_RBBM_SEL_0", "RBBM_PERFCTR_RBBM_0_HI", "RBBM_PERFCTR_RBBM_0_HI" },
		{ "RBBM_PERFCTR_RBBM_SEL_1", "RBBM_PERFCTR_RBBM_1_HI", "RBBM_PERFCTR_RBBM_1_HI" },
		{ "RBBM_PERFCTR_RBBM_SEL_2", "RBBM_PERFCTR_RBBM_2_HI", "RBBM_PERFCTR_RBBM_2_HI" },
		{ "RBBM_PERFCTR_RBBM_SEL_3", "RBBM_PERFCTR_RBBM_3_HI", "RBBM_PERFCTR_RBBM_3_HI" },
	}},
	/* NOTE.. also RBBM_ALWAYSON_COUNTER_HI/RBBM_ALWAYSON_COUNTER_LO, which has
	 * no select register..
	 */
	{ "SP", "a5xx_sp_perfcounter_select", 12, {
		{ "SP_PERFCTR_SP_SEL_0",  "RBBM_PERFCTR_SP_0_HI",  "RBBM_PERFCTR_SP_0_LO" },
		{ "SP_PERFCTR_SP_SEL_1",  "RBBM_PERFCTR_SP_1_HI",  "RBBM_PERFCTR_SP_1_LO" },
		{ "SP_PERFCTR_SP_SEL_2",  "RBBM_PERFCTR_SP_2_HI",  "RBBM_PERFCTR_SP_2_LO" },
		{ "SP_PERFCTR_SP_SEL_3",  "RBBM_PERFCTR_SP_3_HI",  "RBBM_PERFCTR_SP_3_LO" },
		{ "SP_PERFCTR_SP_SEL_4",  "RBBM_PERFCTR_SP_4_HI",  "RBBM_PERFCTR_SP_4_LO" },
		{ "SP_PERFCTR_SP_SEL_5",  "RBBM_PERFCTR_SP_5_HI",  "RBBM_PERFCTR_SP_5_LO" },
		{ "SP_PERFCTR_SP_SEL_6",  "RBBM_PERFCTR_SP_6_HI",  "RBBM_PERFCTR_SP_6_LO" },
		{ "SP_PERFCTR_SP_SEL_7",  "RBBM_PERFCTR_SP_7_HI",  "RBBM_PERFCTR_SP_7_LO" },
		{ "SP_PERFCTR_SP_SEL_8",  "RBBM_PERFCTR_SP_8_HI",  "RBBM_PERFCTR_SP_8_LO" },
		{ "SP_PERFCTR_SP_SEL_9",  "RBBM_PERFCTR_SP_9_HI",  "RBBM_PERFCTR_SP_9_LO" },
		{ "SP_PERFCTR_SP_SEL_10", "RBBM_PERFCTR_SP_10_HI", "RBBM_PERFCTR_SP_10_LO" },
		{ "SP_PERFCTR_SP_SEL_11", "RBBM_PERFCTR_SP_11_HI", "RBBM_PERFCTR_SP_11_LO" },
	}},
	{ "TP", "a5xx_tp_perfcounter_select", 8, {
		{ "TPL1_PERFCTR_TP_SEL_0", "RBBM_PERFCTR_TP_0_HI", "RBBM_PERFCTR_TP_0_LO" },
		{ "TPL1_PERFCTR_TP_SEL_1", "RBBM_PERFCTR_TP_1_HI", "RBBM_PERFCTR_TP_1_LO" },
		{ "TPL1_PERFCTR_TP_SEL_2", "RBBM_PERFCTR_TP_2_HI", "RBBM_PERFCTR_TP_2_LO" },
		{ "TPL1_PERFCTR_TP_SEL_3", "RBBM_PERFCTR_TP_3_HI", "RBBM_PERFCTR_TP_3_LO" },
		{ "TPL1_PERFCTR_TP_SEL_4", "RBBM_PERFCTR_TP_4_HI", "RBBM_PERFCTR_TP_4_LO" },
		{ "TPL1_PERFCTR_TP_SEL_5", "RBBM_PERFCTR_TP_5_HI", "RBBM_PERFCTR_TP_5_LO" },
		{ "TPL1_PERFCTR_TP_SEL_6", "RBBM_PERFCTR_TP_6_HI", "RBBM_PERFCTR_TP_6_LO" },
		{ "TPL1_PERFCTR_TP_SEL_7", "RBBM_PERFCTR_TP_7_HI", "RBBM_PERFCTR_TP_7_LO" },
	}},
	{ "UCHE", "a5xx_uche_perfcounter_select", 8, {
		{ "UCHE_PERFCTR_UCHE_SEL_0", "RBBM_PERFCTR_UCHE_0_HI", "RBBM_PERFCTR_UCHE_0_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_1", "RBBM_PERFCTR_UCHE_1_HI", "RBBM_PERFCTR_UCHE_1_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_2", "RBBM_PERFCTR_UCHE_2_HI", "RBBM_PERFCTR_UCHE_2_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_3", "RBBM_PERFCTR_UCHE_3_HI", "RBBM_PERFCTR_UCHE_3_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_4", "RBBM_PERFCTR_UCHE_4_HI", "RBBM_PERFCTR_UCHE_4_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_5", "RBBM_PERFCTR_UCHE_5_HI", "RBBM_PERFCTR_UCHE_5_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_6", "RBBM_PERFCTR_UCHE_6_HI", "RBBM_PERFCTR_UCHE_6_LO" },
		{ "UCHE_PERFCTR_UCHE_SEL_7", "RBBM_PERFCTR_UCHE_7_HI", "RBBM_PERFCTR_UCHE_7_LO" },
	}},
	{ "VFD", "a5xx_vfd_perfcounter_select", 8, {
		{ "VFD_PERFCTR_VFD_SEL_0", "RBBM_PERFCTR_VFD_0_HI", "RBBM_PERFCTR_VFD_0_LO" },
		{ "VFD_PERFCTR_VFD_SEL_1", "RBBM_PERFCTR_VFD_1_HI", "RBBM_PERFCTR_VFD_1_LO" },
		{ "VFD_PERFCTR_VFD_SEL_2", "RBBM_PERFCTR_VFD_2_HI", "RBBM_PERFCTR_VFD_2_LO" },
		{ "VFD_PERFCTR_VFD_SEL_3", "RBBM_PERFCTR_VFD_3_HI", "RBBM_PERFCTR_VFD_3_LO" },
		{ "VFD_PERFCTR_VFD_SEL_4", "RBBM_PERFCTR_VFD_4_HI", "RBBM_PERFCTR_VFD_4_LO" },
		{ "VFD_PERFCTR_VFD_SEL_5", "RBBM_PERFCTR_VFD_5_HI", "RBBM_PERFCTR_VFD_5_LO" },
		{ "VFD_PERFCTR_VFD_SEL_6", "RBBM_PERFCTR_VFD_6_HI", "RBBM_PERFCTR_VFD_6_LO" },
		{ "VFD_PERFCTR_VFD_SEL_7", "RBBM_PERFCTR_VFD_7_HI", "RBBM_PERFCTR_VFD_7_LO" },
	}},
	{ "VPC", "a5xx_vpc_perfcounter_select", 4, {
		{ "VPC_PERFCTR_VPC_SEL_0", "RBBM_PERFCTR_VPC_0_HI", "RBBM_PERFCTR_VPC_0_LO" },
		{ "VPC_PERFCTR_VPC_SEL_1", "RBBM_PERFCTR_VPC_1_HI", "RBBM_PERFCTR_VPC_1_LO" },
		{ "VPC_PERFCTR_VPC_SEL_2", "RBBM_PERFCTR_VPC_2_HI", "RBBM_PERFCTR_VPC_2_LO" },
		{ "VPC_PERFCTR_VPC_SEL_3", "RBBM_PERFCTR_VPC_3_HI", "RBBM_PERFCTR_VPC_3_LO" },
	}},
	{ "VSC", "a5xx_vsc_perfcounter_select", 2, {
		{ "VSC_PERFCTR_VSC_SEL_0", "RBBM_PERFCTR_VSC_0_HI", "RBBM_PERFCTR_VSC_0_LO" },
		{ "VSC_PERFCTR_VSC_SEL_1", "RBBM_PERFCTR_VSC_1_HI", "RBBM_PERFCTR_VSC_1_LO" },
	}},
	{ "VBIF", "a5xx_vbif_perfcounter_select", 4, {
		{ "VBIF_PERF_CNT_SEL0", "VBIF_PERF_CNT_HIGH0", "VBIF_PERF_CNT_LOW0", "VBIF_PERF_CNT_EN0", "VBIF_PERF_CNT_CLR0" },
		{ "VBIF_PERF_CNT_SEL1", "VBIF_PERF_CNT_HIGH1", "VBIF_PERF_CNT_LOW1", "VBIF_PERF_CNT_EN1", "VBIF_PERF_CNT_CLR1" },
		{ "VBIF_PERF_CNT_SEL2", "VBIF_PERF_CNT_HIGH2", "VBIF_PERF_CNT_LOW2", "VBIF_PERF_CNT_EN2", "VBIF_PERF_CNT_CLR2" },
		{ "VBIF_PERF_CNT_SEL3", "VBIF_PERF_CNT_HIGH3", "VBIF_PERF_CNT_LOW3", "VBIF_PERF_CNT_EN3", "VBIF_PERF_CNT_CLR3" },
	}},
};

/*
 * main
 */

int
main(int argc, char **argv)
{
	find_device();

	/* load corresponding rnn db, etc: */
	rnn_init();
	struct rnndb *db = rnn_newdb();
	rnn_parsefile(db, "adreno.xml");
	rnn_prepdb(db);
	dev.ctx = rnndec_newcontext(db);
	dev.ctx->colors = &envy_null_colors;

	switch (dev.chipid >> 24) {
	case 3:
		dev.dom = rnn_finddomain(db, "A3XX");
		dev.groups = a3xx_counters;
		dev.ngroups = ARRAY_SIZE(a3xx_counters);
		break;
	case 4:
		dev.dom = rnn_finddomain(db, "A4XX");
		dev.groups = a4xx_counters;
		dev.ngroups = ARRAY_SIZE(a4xx_counters);
		break;
	case 5:
		dev.dom = rnn_finddomain(db, "A5XX");
		dev.groups = a5xx_counters;
		dev.ngroups = ARRAY_SIZE(a5xx_counters);
		break;
	default:
		errx(1, "unsupported device");
	}

	setup_counter_groups();

	config_restore();

	main_ui();

	return 0;
}
