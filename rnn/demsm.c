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

		if (!fgets(buf, sizeof(buf), fin))
			break;

		if (!strncmp(buf, "region", 6)) {
			int i = domains_count++;
			sscanf(buf, "region %s %x %x", domains[i].name,
					&domains[i].base, &domains[i].size);
			printf("domain: %s %08x/%08x\n", domains[i].name,
					domains[i].base, domains[i].size);
			domains[i].dom = rnn_finddomain(db, domains[i].name);
		} else {
			char func[256];
			uint32_t ts, current, op, addr, val, line;

			if (sscanf(buf, "%x %x %d %x %x %s %d", &ts, &current, &op,
					&addr, &val, func, &line) == 7) {
				struct domain *d = find_domain(addr);

				if (d && d->dom) {
					struct rnndecaddrinfo *ai = rnndec_decodeaddr(ctx, d->dom, addr - d->base, op);
					char *decoded_val = rnndec_decodeval(ctx, ai->typeinfo, val, ai->width);
					printf("%08x %08x %7s:%-30s %c %s", ts, current,
							d->dom->name, ai->name,
							op ? '<' : '>', decoded_val);
					free(ai->name);
					free(ai);
					free(decoded_val);
				} else {
					printf("%08x %08x %08x %c %08x", ts, current,
							addr, op ? '<' : '>', val);
				}

				if (verbose) {
					printf("\t\t%s\t%d", func, line);
				}

				printf("\n");
			} else {
				printf("%s", buf);
			}
		}
	}

	return 0;
}
