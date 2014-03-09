/*
 * Copyright (C) 2010-2011 Marcin Kościelnicki <koriakin@0x04.net>
 * Copyright (C) 2011 Martin Peres <martin.peres@ensi-bourges.fr>
 * Copyright (C) 2013 Rob Clark <robclark@freedesktop.org>
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

#include "rnn.h"
#include "rnndec.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <getopt.h>

struct domain {
	char name[16];
	struct rnndomain *dom;
	uint32_t base, size;
	int shift;
	char *writemask;
	uint32_t *val;
};

static struct domain domains[16];
static int domains_count = 0;


static int is_a2xx(const char *name)
{
	return !strcmp("A225", name) ||
			!strcmp("A220", name) ||
			!strcmp("A205", name) ||
			!strcmp("A2XX", name);
}

static int is_a3xx(const char *name)
{
	return !strcmp("A330", name) ||
			!strcmp("A320", name) ||
			!strcmp("A305", name) ||
			!strcmp("A3XX", name);
}

static struct domain *find_domain(struct rnndeccontext *ctx, uint32_t *addrp)
{
	struct domain *first_domain = NULL; /* if no exact match, pick first closest match */
	uint32_t addr = *addrp;
	int i;
	for (i = 0; i < domains_count; i++) {
		struct domain *d = &domains[i];

		if ((d->base <= addr) && (addr < (d->base + d->size))) {
			if (!d->dom)
				continue;
			if (!first_domain)
				first_domain = d;
			if (rnndec_checkaddr(ctx, d->dom, (addr - d->base) >> d->shift, 0)) {
				return d;
			}
		}
	}
	if (first_domain && is_a3xx(first_domain->name)) {
		/* we appear to have some banked registers: */
		uint32_t off = addr - first_domain->base;
		if ((0x9000 <= off) && (off < 0x10000)) {
			*addrp -= 0x1000;
			return find_domain(ctx, addrp);
		}
	}
	return first_domain;
}

static int find_region(const char *buf, char *name, uint32_t *base, uint32_t *size)
{
	const char *str;

	/* kernel dmesg style (skips timestamp if present): */
	str = strstr(buf, "IO:region");
	if (str)
		return sscanf(str, "IO:region %16s %x %x", name, base, size) == 3;

	/* debugfs log style: */
	return sscanf(buf, "region %16s %x %x", name, base, size) == 3;
}

static int find_reg(const char *buf, int *n, int *m,
		uint32_t *op, uint32_t *addr, uint32_t *val)
{
	const char *str;
	int ret;

	/* kernel dmesg style (skips timestamp if present): */
	str = strstr(buf, "IO:R");
	if (str) {
		*op = 0;
		*n = (str - buf);
		ret = sscanf(str, "IO:R %x %x%n", addr, val, m);
		*m += *n;
		return ret == 2;
	}
	str = strstr(buf, "IO:W");
	if (str) {
		*op = 1;
		*n = (str - buf);
		ret = sscanf(str, "IO:W %x %x%n", addr, val, m);
		*m += *n;
		return ret == 2;
	}

	/* debugfs log style: */
	return sscanf(buf, "%*x %*x %n%d %x %x%n", n, op, addr, val, m) == 3;
}

const char *domain_suffixes[] = {
		"8960", "8x60", NULL
};

static void printval(struct rnndeccontext *ctx, uint32_t addr, uint32_t val, uint32_t op)
{
	uint32_t origaddr = addr;
	struct domain *d = find_domain(ctx, &addr);
	if (d && d->dom) {
		uint32_t off = addr - d->base;
		struct rnndecaddrinfo *ai = rnndec_decodeaddr(ctx, d->dom, off >> d->shift, op);
		char *decoded_val = rnndec_decodeval(ctx, ai->typeinfo, val, ai->width);
		if (origaddr != addr) {
			printf("!%9s:%-30s %s", d->dom->name, ai->name, decoded_val);
		} else {
			printf("%10s:%-30s %s", d->dom->name, ai->name, decoded_val);
		}
		free(ai->name);
		free(ai);
		free(decoded_val);

		if (op == 1) { /* write */
			uint32_t idx = off/4;

			if (!d->writemask) {
				d->writemask = calloc(1, d->size / 8);
				d->val = calloc(1, d->size);
			}

			d->writemask[idx / 8] |= 1 << (idx % 8);
			d->val[idx] = val;
		}
	} else {
		printf("%08x %08x", addr, val);
	}
}

int main(int argc, char **argv) {
	char *file = NULL;
	int c,use_colors=1,verbose=0;
	int j, i;

	while ((c = getopt (argc, argv, "f:cv")) != -1) {
		switch (c) {
			case 'f':{
				file = strdup(optarg);
				break;
			}
			case 'c':{
				use_colors = 0;
				break;
			}
			case 'v':{
				verbose = 1;
				break;
			}
			default:{
				break;
			}
		}
	}
	rnn_init();

	struct rnndb *db = rnn_newdb();
	rnn_parsefile(db, "msm.xml");
	rnn_parsefile(db, "adreno.xml");
	rnn_prepdb(db);
	struct rnndeccontext *ctx = rnndec_newcontext(db);
	ctx->colors = use_colors ? &envy_def_colors : &envy_null_colors;

	FILE *fin = (file==NULL) ? stdin : fopen(file, "r");;
	if (!fin) {
		fprintf (stderr, "Failed to open input file!\n");
		return 1;
	}

	rnndec_varadd(ctx, "chipset", "MDP40");

	while (1) {
		char buf[1024];

		i = domains_count;

		if (!fgets(buf, sizeof(buf), fin))
			break;

		if (find_region(buf, domains[i].name, &domains[i].base, &domains[i].size)) {
			printf("%s", buf);
			/* special handling for gpu: */
			if (is_a3xx(domains[i].name) || is_a2xx(domains[i].name)) {
				if (is_a3xx(domains[i].name)) {
					sprintf(domains[i].name, "A3XX");
					domains[i].dom = rnn_finddomain(db, domains[i].name);
				} else {
					sprintf(domains[i].name, "A2XX");
					domains[i].dom = rnn_finddomain(db, domains[i].name);
				}
				domains[i].shift = 2;
				i = ++domains_count;
				domains[i] = domains[i-1];
				sprintf(domains[i].name, "AXXX");
			}
			domains[i].dom = rnn_finddomain(db, domains[i].name);
			i = ++domains_count;

			/* attempt to load hw specific domains: */
			for (j = 0; domain_suffixes[j]; j++) {
				domains[i] = domains[i-1];
				sprintf(domains[i].name, "%s_%s", domains[i-1].name, domain_suffixes[j]);
				domains[i].dom = rnn_finddomain(db, domains[i].name);
				if (domains[i].dom)
					i = ++domains_count;
			}
		} else {
			int n, m;
			uint32_t op, addr, val;

			if (find_reg(buf, &n, &m, &op, &addr, &val)) {

				printf("%.*s %s%c%s ", n, buf,
						ctx->colors->regsp,
						(op == 1) ? 'W' : 'R',
						ctx->colors->reset);

				printval(ctx, addr, val, op);

				if (verbose) {
					printf("\t\t%s", buf + m);
				} else {
					printf("\n");
				}
			} else {
				printf("%s", buf);
			}
		}
	}

	printf("WRITTEN REGISTER SUMMARY\n");
	for (i = 0; i < domains_count; i++) {
		struct domain *d = &domains[i];
		if (d->writemask) {
			for (j = 0; j < d->size; j++) {
				if (d->writemask[j / 8] & (1 << (j % 8))) {
					uint32_t off = j * 4;
					uint32_t addr = off + d->base;
					printval(ctx, addr, d->val[j], 0);
					printf("\n");
				}
			}
		}
	}

	return 0;
}
