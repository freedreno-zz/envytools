-- Parse cmdstream dump and analyse blits and batches

--local posix = require "posix"

function printf(fmt, ...)
	return io.write(string.format(fmt, ...))
end

function dbg(fmt, ...)
	--printf(fmt, ...)
end

printf("Analyzing Data...\n")

local r = rnn.init("a630")

-- Each submit, all draws will target the same N MRTs:
local mrts = {}
local allmrts = {}  -- includes historical render targets
function push_mrt(fmt, w, h, base, gmem)
	dbg("MRT: %s %ux%u 0x%x\n", fmt, w, h, base)

	local mrt = {}
	mrt.format = fmt
	mrt.w = w
	mrt.h = h
	mrt.base = base
	mrt.gmem = gmem

	mrts[base] = mrt
	allmrts[base] = mrt
end

-- And each each draw will read from M sources/textures:
local sources = {}
function push_source(fmt, w, h, base)
	dbg("SRC: %s %ux%u 0x%x\n", fmt, w, h, base)

	local source = {}
	source.format = fmt
	source.w = w
	source.h = h
	source.base = base

	sources[base] = source
end

local binw
local binh
local nbins
local blits = 0
local draws = 0
local drawmode
local cleared
local restored
local resolved
local nullbatch
local depthtest
local depthwrite
local stenciltest
local stencilwrite

function start_cmdstream(name)
	printf("Parsing %s\n", name)
end

function start_submit()
	dbg("start_submit\n")
	mrts = {}
	sources = {}
	draws = 0
	blits = 0
	cleared = {}
	restored = {}
	resolved = {}
	nullbatch = true
	depthtest = false
	depthwrite = false
	stenciltest = false
	stencilwrite = false
end

function end_submit()
	dbg("end_submit\n")

	printf("\n")

	-- TODO we get false-positives for 'NULL BATCH!' because we don't have
	-- a really good way to differentiate between submits and cmds.  Ie.
	-- with growable cmdstream, and a large # of tiles, IB1 can get split
	-- across multiple buffers.  Since we ignore GMEM draws for window-
	-- offset != 0,0, the later cmds will appear as null batches
	if draws == 0 and blits == 0 then
		if nullbatch then
			printf("NULL BATCH!\n");
		end
		return
	end

	if draws > 0 then
		printf("Batch:\n")
		printf("-------\n")
		printf("  # of draws: %u\n", draws)
		printf("  mode: %s\n", drawmode)
		if drawmode == "RM6_GMEM" then
			printf("  bin size: %ux%u (%u bins)\n", binw, binh, nbins)
		end
		if depthtest or depthwrite then
			printf("  ")
			if depthtest then
				printf("DEPTHTEST ")
			end
			if depthwrite then
				printf("DEPTHWRITE")
			end
			printf("\n")
		end
		if stenciltest or stencilwrite then
			printf("  ")
			if stenciltest then
				printf("STENCILTEST ")
			end
			if stencilwrite then
				printf("STENCILWRITE")
			end
			printf("\n")
		end
	else
		printf("Blit:\n")
		printf("-----\n")
	end

	for base,mrt in pairs(mrts) do
		printf("  MRT[0x%x]:\t%ux%u\t\t%s", base, mrt.w, mrt.h, mrt.format)
		if drawmode == "RM6_GMEM" then
			if cleared[mrt.gmem] then
				printf("\tCLEARED")
			end
			if restored[mrt.gmem] then
				printf("\tRESTORED")
			end
			if resolved[mrt.gmem] then
				printf("\tRESOLVED")
			end
		end
		printf("\n")
	end

	for base,source in pairs(sources) do
		-- only show sources that have been previously rendered to, other
		-- textures are less interesting.  Possibly this should be an
		-- option somehow
		if allmrts[base] then
			printf("  SRC[0x%x]:\t%ux%u\t\t%s\n", base, source.w, source.h, source.format)
		end
	end
end

-- Track the current mode:
local mode = ""
function CP_SET_MARKER(pkt, size)
	mode = pkt[0].MODE
	dbg("mode: %s\n", mode)
end

function CP_EVENT_WRITE(pkt, size)
	if tostring(pkt[0].EVENT) ~= "BLIT" then
		return
	end
	nullbatch = false
	local m = tostring(mode)
	if m == "RM6_GMEM" then
		-- either clear or restore:
		if r.RB_BLIT_INFO.CLEAR_MASK == 0 then
			restored[r.RB_BLIT_BASE_GMEM] = 1
		else
			cleared[r.RB_BLIT_BASE_GMEM] = 1
		end
	else
		if m ~= "RM6_RESOLVE" then
			printf("I am confused!!!\n")
			return
		end
		resolved[r.RB_BLIT_BASE_GMEM] = 1
	end
end

function A6XX_TEX_CONST(pkt, size)
	push_source(pkt[0].FMT,
		pkt[1].WIDTH, pkt[1].HEIGHT,
		pkt[4].BASE_LO | (pkt[5].BASE_HI << 32))
end

function handle_blit()
	-- This kinda assumes that we are doing full img blits, which is maybe
	-- Not completely legit.  We could perhaps instead just track pitch and
	-- size/pitch??  Or maybe the size doesn't matter much
	push_mrt(r.RB_2D_DST_INFO.COLOR_FORMAT,
		r.GRAS_2D_DST_BR.X + 1,
		r.GRAS_2D_DST_BR.Y + 1,
		r.RB_2D_DST_LO | (r.RB_2D_DST_HI << 32),
		-1)
	push_source(r.SP_2D_SRC_FORMAT.COLOR_FORMAT,
		r.GRAS_2D_SRC_BR_X.X + 1,
		r.GRAS_2D_SRC_BR_Y.Y + 1,
		r.SP_PS_2D_SRC_LO | (r.SP_PS_2D_SRC_HI << 32))
	blits = blits + 1
end

function draw(primtype, nindx)
	dbg("draw: %s (%s)\n", primtype, mode)
	nullbatch = false
	if primtype == "BLIT_OP_SCALE" then
		handle_blit()
		return
	end
	local m = tostring(mode)
	if m ~= "RM6_GMEM" and m ~= "RM6_BYPASS" then
		return
	end

	-- Only count the first tile for GMEM mode to avoid counting
	-- each draw for each tile
	if m == "RM6_GMEM" then
		if r.RB_WINDOW_OFFSET.X ~= 0 or r.RB_WINDOW_OFFSET.Y ~= 0 then
			return
		end
	end

	drawmode = m
	for n = 0,r.RB_FS_OUTPUT_CNTL1.MRT-1 do
		push_mrt(r.RB_MRT[n].BUF_INFO.COLOR_FORMAT,
			r.GRAS_SC_SCREEN_SCISSOR_BR_0.X + 1,
			r.GRAS_SC_SCREEN_SCISSOR_BR_0.Y + 1,
			r.RB_MRT[n].BASE_LO | (r.RB_MRT[n].BASE_HI << 32),
			r.RB_MRT[n].BASE_GMEM)
	end

	local depthbase = r.RB_DEPTH_BUFFER_BASE_LO |
			(r.RB_DEPTH_BUFFER_BASE_HI << 32)

	if depthbase ~= 0 then
		push_mrt(r.RB_DEPTH_BUFFER_INFO.DEPTH_FORMAT,
			r.GRAS_SC_SCREEN_SCISSOR_BR_0.X + 1,
			r.GRAS_SC_SCREEN_SCISSOR_BR_0.Y + 1,
			depthbase,
			r.RB_DEPTH_BUFFER_BASE_GMEM)
	end

	if r.RB_DEPTH_CNTL.Z_WRITE_ENABLE then
		depthwrite = true
	end

	if r.RB_DEPTH_CNTL.Z_ENABLE then
		depthtest = true
	end

	-- clearly 0 != false.. :-/
	if r.RB_STENCILWRMASK.WRMASK ~= 0 then
		stencilwrite = true
	end

	if r.RB_STENCIL_CONTROL.STENCIL_ENABLE then
		stenciltest = true
	end

	-- TODO should also check for stencil buffer for z32+s8 case

	if m == "RM6_GMEM" then
		binw = r.VSC_BIN_SIZE.WIDTH
		binh = r.VSC_BIN_SIZE.HEIGHT
		nbins = r.VSC_BIN_COUNT.NX * r.VSC_BIN_COUNT.NY
	end

	draws = draws + 1
end

