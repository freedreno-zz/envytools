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
};

static struct domain domains[16];
static int domains_count = 0;

static struct domain *find_domain(uint32_t addr)
{
	int i;
	for (i = 0; i < domains_count; i++)
		if ((domains[i].base <= addr) && (addr < (domains[i].base + domains[i].size)))
			return &domains[i];
	return NULL;
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

int main(int argc, char **argv) {
	char *file = NULL;
	int c,use_colors=1,verbose=0;
	while ((c = getopt (argc, argv, "f:c:v")) != -1) {
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
		int i = domains_count;

		if (!fgets(buf, sizeof(buf), fin))
			break;

		if (find_region(buf, domains[i].name, &domains[i].base, &domains[i].size)) {
			printf("%s", buf);
			domains[i].dom = rnn_finddomain(db, domains[i].name);
			domains_count++;
		} else {
			int n, m;
			uint32_t op, addr, val;

			if (find_reg(buf, &n, &m, &op, &addr, &val)) {
				struct domain *d = find_domain(addr);

				printf("%.*s %s%c%s ", n, buf,
						ctx->colors->regsp,
						(op == 1) ? 'W' : 'R',
						ctx->colors->reset);

				if (d && d->dom) {
					struct rnndecaddrinfo *ai = rnndec_decodeaddr(ctx, d->dom, addr - d->base, op);
					char *decoded_val = rnndec_decodeval(ctx, ai->typeinfo, val, ai->width);
					printf("%7s:%-30s %s", d->dom->name, ai->name, decoded_val);
					free(ai->name);
					free(ai);
					free(decoded_val);
				} else {
					printf("%08x %08x", addr, val);
				}

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

	return 0;
}
