// Copyright 2020 Steven A. Falco <stevenfalco@gmail.com>
//
//  ezparse is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  ezparse is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with ezparse.  If not, see <https://www.gnu.org/licenses/>.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <complex.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#define STR_LEN			16
#define TLEN			30

#define PRIMARY_BS		170 // size of primary blocks

#define VB_FREQ_SWEEP		11
#define VB_WIRE_INS		12
#define VB_TL_LOSS		14
#define VB_TRANSFORMER		15
#define VB_Y_PARAM		16
#define VB_L_NETWORK		17
#define VB_VIRTUAL_SEG		18
#define VB_PLANE_WAVE_SRC	31

#define VSEG_MAX		1000

#define SHORTED_END		0xffff
#define OPEN_END		0xfffe


// EZNEC Pro allows 20000 segments, so there cannot be more
// wires than that.
#define WIRE_MAX		20000

// Cast a void pointer to a char pointer, add an offset, then cast it to the
// desired type.  While gcc would let us add a constant to a void pointer
// directly, it is not portable.
#define OFFSET_TO(type, void_pointer, offset_in_bytes) ((type *)(((char *)(void_pointer)) + (offset_in_bytes)))

// Map a pointer (lval) of type (type) to a location defined as void_pointer + offset_in_bytes.
// Do some range checking and debug printing as well.
//
// This is used for fixed-length blocks.
#define MAP(lval, type, void_pointer, offset_in_bytes)					\
	do {										\
		if(gDebug) fprintf(stderr, "map " #type " at 0x%x through 0x%x\n",	\
				offset_in_bytes, offset_in_bytes + sizeof(type) - 1);	\
		if((offset_in_bytes + sizeof(type)) > gInputFileSize) {			\
			fprintf(stderr, "Cannot map - runs off end of file\n");		\
			exit(1);							\
		}									\
		(lval) = OFFSET_TO(type, (void_pointer), (offset_in_bytes));		\
	} while(0)

// Map a pointer (lval) of type (type) to a location defined as void_pointer + offset_in_bytes.
// Do some range checking and debug printing as well.
//
// This is used for variable-length blocks.
#define VMAP(lval, type, void_pointer, offset_in_bytes, length)				\
	do {										\
		if(gDebug) fprintf(stderr, "map " #type " at 0x%x through 0x%x\n",	\
				offset_in_bytes, offset_in_bytes + length - 1);		\
		if((offset_in_bytes + length) > gInputFileSize) {			\
			fprintf(stderr, "Cannot map - runs off end of file\n");		\
			exit(1);							\
		}									\
		(lval) = OFFSET_TO(type, (void_pointer), (offset_in_bytes));		\
	} while(0)

int	gDebug = 0;

// NB: We only declare structs to be packed if they really need it.
// Otherwise, the compiler would give us "unaligned pointer" warnings
// in some cases.
//
// Also, we sometimes split blocks to separate items of the same type.

// RecType1 - globals
//
// 170 bytes long
typedef struct __attribute__((__packed__)) {
	uint16_t	OldMaxMWSL;	// Old Maximum of NW, NSrc, NL, NT, NM
	uint16_t	NM;		// GN Number of media, usually 1, occasionally 2
	uint8_t		BdryType;	// GN Set to 'R' if media2 is radials, else 'X'
	float		FMHz;		// FR FMHZ, starting frequency or single frequency
	uint8_t		OldPType;	// (A)zimuth or (E)levation; write ‘E’ if plot type is 3D
	float		Pangle;		// RP Plot angle - could be azimuth or elevation
	float		PStep;		// RP Plot angle steps
	char		Title[TLEN];	// CM Title
	uint16_t	OldNW;		// old GW Number of wires
	uint16_t	NSrc;		// EX Number of sources
	uint16_t	NL;		// LD Number of loads
	uint8_t		Gtype;		// GN Ground type 'P' perfect, 'R' real, 'F' free space
	uint16_t	NR;		// Number of radials
	float		RDia;		// Radial diameter
	uint16_t	RDiaGa;		// Radial wire gauge - negative means "ignore"
	uint16_t	Ck;		// Check Constant = 1304 always
	uint8_t		Units;		// M meters, L millimeters, F feet, I inches, W wavelength
	uint8_t		PRange;		// 2D Plot range: (F)ull or (P)artial
	uint8_t		OldPPol;	// Polarizations. shown in 2D plot: (T)ot only, or (A)ll [VHT]
	float		ORdb;		// dBi of outer ring of 2D plot (1304 = auto scaled)
	float		PStart;		// Start angle for 2D plot in degrees
	float		PEnd;		// End angle for 2D plot in degrees
	uint8_t		OldLType;	// Load type: S (Laplace) or Z. Write ‘Z’ if load type is RLC
	float		GWDist;		// Ground wave distance (used by EZNEC Pro only)
	float		GWZ;		// Ground wave elevation (z coord) (used by EZNEC Pro only)
	uint8_t		LNetType;	// L Network type: (Z) - RX; (R) - RLC
	float		AnalRes;	// Write the same value into this field as into PStep
	float		RefdB;		// Field strength reference level in dBi
	float		WRho;		// Wire resistivity in ohm-m
	float		WMu;		// Wire relative permeability
	uint16_t	NP;		// 0 - used by earlier programs
	uint16_t	ArrayFiles;	// 0 - used by earlier programs
	float		SWRZ0;		// User-defined Z0 for SWR outputs
	uint16_t	VerCode;	// Writing program version code: Write decimal 51 into this field
	uint8_t		DiaUnits;	// Diameter units, usually M or L millimeters, F or I inches, W wavelength
	uint8_t		LType;		// Load type: (Z) - RX; (R) - RLC; (L) - Laplace
	uint16_t	PPol;		// 2D plot polarizations shown - 1-Tot only; 2-V,H,T; 3-V,H; 4-R & L circ, 5-R circ, 6-L circ, 7-Lin Maj/Min
	uint16_t	NT;		// TL Number of transmission lines
	uint8_t		PType;		// Plot type: (A)z, (E)l, (2)D or (3)D
	uint8_t		GAnal;		// Real gnd analysis type: (H)igh accuracy, (M)ININEC
	uint32_t	MaxMWSL;	// Maximum of Nw, NSrc, NL, NT, NM
	uint32_t	NW;		// GW Number of wires
	float		dBSum;		// Always write 0
	float		PStep3D;	// 3D plot step size
	uint32_t	MiscFlags;	// Bit 0 =  Allow ctr of seg within another wire, Bit 1 =  allow plane wave excitation with no conventional sources
	uint32_t	PgmVerCode;	// 50000200 old, or 62000000 currently
	float		LinRange2D;	// Range of 2D linear plot scale
	uint16_t	NX;		// Number of transformers
	uint16_t	NN;		// Number of Y parameter networks
	uint16_t	NLNet;		// Number of LNets
	uint16_t	NVirt;		// Number of Virtual segments
	uint16_t	Reserved1;	// Write 0
	uint8_t		HAGndType;	// “High” accuracy gnd type (H)igh or e(X)tended
	char		Reserved2[7];	// Write 0
} RecType1;

// RecType2 - per-wire/source/load etc.
//
// Also has some overflow from RecType1, so some things get repeated that
// don't need to be.
//
// 170 bytes long
typedef struct __attribute__((__packed__)) {
	// Medium (ground) parameters.
	float		MSigma;		// GN SIG (conductivity in Siemens, 0.005 is typical)
	float		MEps;		// GN EPSR (dielectric constant, 13 is typical)
	float		MCoord;		// GN Medium R or X coordinate of medium
	float		MHt;		// GN Medium height

	// Wire parameters.
	float		WEnd1_X;	// GW end 1 X
	float		WEnd1_Y;	// GW end 1 Y
	float		WEnd1_Z;	// GW end 1 Z
	float		WEnd2_X;	// GW end 2 X
	float		WEnd2_Y;	// GW end 2 Y
	float		WEnd2_Z;	// GW end 2 Z
	float		WireDia;	// GW wire diameter
	uint16_t	WDiaGa;		// GW wire gauge (negative if not specified as gauge)
	uint16_t	WSegs;		// GW wire segments

	// Source parameters.
	uint16_t	SWNr;		// EX source wire number
	float		SWPct;		// EX source wire percentage from end 1 (position on wire)
	float		SMP_M;		// EX source magnitude, volts or amperes
	float		SMP_Pdeg;	// EX source phase angle in degrees
	uint8_t		Stype;		// EX source type: V voltage, I current, W split voltage, J split current

	// Load parameters.
	uint16_t	LWNr;		// LD load wire number
	float		LWPct;		// LD load wire percentage from end 1(position on wire)
	float		LZ_R;		// LD load resistance (real part)
	float		LZ_I;		// LD load reactance (imaginary part)
	float		LNum_S0;	// 0-order numerator coeff. of Laplace load impedance
	float		LNum_S1;	// 1-order numerator coeff. of Laplace load impedance
	float		LNum_S2;	// 2-order numerator coeff. of Laplace load impedance
	float		LNum_S3;	// 3-order numerator coeff. of Laplace load impedance
	float		LNum_S4;	// 4-order numerator coeff. of Laplace load impedance
	float		LNum_S5;	// 5-order numerator coeff. of Laplace load impedance
	float		LDen_S0;	// 0-order denominator coeff. of Laplace load impedance
	float		LDen_S1;	// 1-order denominator coeff. of Laplace load impedance
	float		LDen_S2;	// 2-order denominator coeff. of Laplace load impedance
	float		LDen_S3;	// 3-order denominator coeff. of Laplace load impedance
	float		LDen_S4;	// 4-order denominator coeff. of Laplace load impedance
	float		LDen_S5;	// 5-order denominator coeff. of Laplace load impedance

	// Transmission line parameters.
	uint16_t	TLWNr1;		// TL wire #1 endpoint, or -1 (shorted stub) or -2 (open stub)
	float		TLWPct1;	// TL wire #1 percentage from end 1 (position on wire)
	uint16_t	TLWNr2;		// TL wire #2 endpoint, or -1 (shorted stub) or -2 (open stub)
	float		TLWPct2;	// TL wire #2 percentage from end 1 (position on wire)
	float		TLZ0;		// TL impedance, Z0, negated if reversed connection
	float		TLLen;		// TL length, 0 => physical length; neg => degrees; pos => meters
	float		TLVF;		// TL velocity factor

	// Load parameters (additional).
	uint8_t		LConn;		// LD connection type: S = series, P = parallel

	// Load parameters (only for RLC loads - LType == 'R').
	uint8_t		strRLCType;	// LD S = series, P = parallel, T = trap
	float		sngR;		// LD Resistance in Ohms (0 means "not present")
	float		sngL;		// LD Inductance in Henries (0 means "not present")
	float		sngC;		// LD Capacitance in Farads (0 means "not present")
	float		sngRFreqMHz;	// LD Frequency
	char		Reserved[3];	// Write zeros
} RecType2;

// RecType3 - Near Field parameters
//
// 170 bytes long
typedef struct __attribute__((__packed__)) {
	uint8_t		NFType;		// E or H, electrical field vs magnetic field
	uint8_t		NFCoordType;	// S (spherical) or C (cartesian)
	float		NF1Start;	// Starting X or rho
	float		NF1Stop;	// Stopping X or rho
	float		NF1Step;	// Step size X or rho
	float		NF2Start;	// Starting Y or theta (azimuth in degrees)
	float		NF2Stop;	// Stopping Y or theta (azimuth in degrees)
	float		NF2Step;	// Step size Y or theta
	float		NF3Start;	// Starting Z or phi (zenith in degrees)
	float		NF3Stop;	// Stopping Z or phi (zenith in degrees)
	float		NF3Step;	// Step size Z or phi
	char		Reserved[132];	// Write zeros
} RecType3;

// RecType4 - reserved
//
// 170 bytes long
typedef struct __attribute__((__packed__)) {
	char		Reserved[170];	// Not used for anything currently
} RecType4;

// RecType4 is the last of the fixed-length records.  All other
// records are variable-length, and all begin with the following
// header.  We don't include the header fields in the individual
// block structures, because that would break field alignment.
//
// We have to read the BlockType and BlockLen to determine the
// block type and length.
//
// NB - there are probably additional BlockType's that we don't know
// about.
typedef struct __attribute__((__packed__)) {
	uint16_t	BlockType;	// Block type 11, 12, 14, 15, 17, 18, 31, 101
	uint32_t	BlockLen;	// Length, includes this header
	uint8_t		BlockRev;	// Block format revision
} BlkHeader;

// Frequency Sweep Block
//
// This block is a bit difficult because it is specified as containing
// a series of variable-length arrays.  Thus, everything after FreqInFile
// has to be calculated rather than using the structure variable names.
//
// We give all the File fields a length of 1, but they may vary from 0 to
// some large number.  Also, the strings are NOT null-terminated.
// 
// WARNING: We must use the BlockLen rather than the sizeof(FreqSweepBlk)!
typedef struct __attribute__((__packed__)) {
	uint32_t	DataLen;	// Set to 16
	uint16_t	FIFLen;		// Length of Frequency Input file path and name
	uint16_t	PPFLen;		// Length of Pattern Plot file path and name
	uint16_t	SCPFLen;	// Length of Smith Chart Program path and name
	uint16_t	DFLen;		// Length of Data file path and name
	uint16_t	AFLen;		// Reserved
	uint32_t	Flags;		// Bits are defined below
	float		FStart;		// Starting frequency
	float		FStop;		// Ending frequency
	float		FStep;		// Frequency step size
	char		FreqInFile[1];	// Frequency Input file path and name
	char		PatPlotFile[1];	// Pattern Plot file path and name
	char		SCPFile[1];	// PSmith Chart Program path and name
	char		DataFile[1];	// PData file path and name
	uint16_t	EndTest;	// Always set to 12345
} FreqSweepBlk;

// "Flags" bits in the FreqSweepBlk.Flags field above.
//
// If bit 3 is set, bits 1 and 10 are ignored.
#define FSB_Save_Pattern_Plots		(1 <<  1)
#define FSB_Save_Field_Strength		(1 <<  2)
#define FSB_Near_Field_Analysis		(1 <<  3)
#define FSB_Save_Source_Data		(1 <<  4)
#define FSB_Read_Freq_From_File		(1 <<  5)
#define FSB_Save_Load_Data		(1 <<  6)
#define FSB_Use_Alt_Z0			(1 <<  7)
#define FSB_Save_Currents		(1 <<  8)
#define FSB_Save_Pattern_Analysis	(1 << 10)
#define FSB_Save_Freq_Sweep_Data	(1 << 13)
#define FSB_Save_Smith_Chart_Prog	(1 << 14)

// Wire Insulation Block
//
// WARNING: We must use the BlockLen rather than the sizeof(WireInsBlock)!
typedef struct __attribute__((__packed__)) {
	uint32_t	NumWires;	// Number of wires in the following array
	struct {
		float		DielC;	// Wire Insulation Dielectric (permittivity)
		float		Thk;	// Wire Insulation Thickness (meters)
		float		LTan;	// Loss Tangent
	} Wires[1];
} WireInsBlock;

// Transmission Line Loss Parameters
//
// This block is a bit difficult because it is specified as containing
// a pair of variable-length arrays.  Thus, everything after NumLines
// has to be calculated rather than using the structure variable names.
//
// We give all the array fields a length of 1, but they may vary from 0 to
// some large number.
typedef struct {
	uint32_t	NumLines;	// Number of transmission lines in the arrays
	float		Loss[1];	// Transmission line loss (dB / 100 feet)
	float		LossFreq[1];	// Transmission line loss frequency
} TLineLossBlock;

// Transformer Parameters
//
// This block consists of three variable-length arrays.  Additionally, the
// arrays are really two-dimensional, because a transformer has two ports.
// There is no way to create a C structure like that, so we instead split
// the block into four pieces (A-D).
//
// Part A contains NX
// Part B contains Wires
// Part C contains position (percentage)
// Part D contains impedance
typedef struct {
	uint32_t	NX;		// Number of transformers
} TransformerBlockA;

typedef struct {
	uint32_t	P1WNr;		// Port 1 wire number
	uint32_t	P2WNr;		// Port 2 wire number
} TransformerBlockB;

typedef struct {
	float		P1WPct;		// Port 1 percentage (position on wire)
	float		P2WPct;		// Port 2 percentage (position on wire)
} TransformerBlockC;

typedef struct {
	float		P1RelZ;		// Port 1 impedance (if negative, reverse polarity)
	float		P2RelZ;		// Port 2 impedance (if negative, reverse polarity
} TransformerBlockD;

typedef struct {
	TransformerBlockA *pA;
	TransformerBlockB *pB;
	TransformerBlockC *pC;
	TransformerBlockD *pD;
} TransformerBlockPtrs;

// L Network Parameters
//
// This block consists of five variable-length arrays.  Additionally, the
// arrays are really two-dimensional, because an L-network has two ports.
// There is no way to create a C structure like that, so we instead split
// the block into seven pieces (A-G).
typedef struct {
	uint32_t	NL;		// Number of L-networks
} LNetBlockA;

typedef struct {
	uint32_t	P1WNr;		// Port 1 wire number
	uint32_t	P2WNr;		// Port 2 wire number
} LNetBlockB;

typedef struct {
	float		P1WPct;		// Port 1 percentage (position on wire)
	float		P2WPct;		// Port 2 percentage (position on wire)
} LNetBlockC;

// Using R +/- jX
typedef struct {
	float		B1R;		// Port 1 resistance
	float		B1X;		// Port 1 reactance
	float		B2R;		// Port 2 resistance
	float		B2X;		// Port 2 reactance
} LNetBlockD;

typedef struct {
	uint32_t	NrRLC;		// WEzT1.NLNet if using RLC, else 0
} LNetBlockE;

// Using RLC at a frequency
typedef struct {
	float		B1rlcR;		// Port 1 resistance
	float		B2rlcR;		// Port 2 resistance
	float		B1rlcL;		// Port 1 inductance (Henries)
	float		B2rlcL;		// Port 2 inductance (Henries)
	float		B1rlcC;		// Port 1 capacitance (Farads)
	float		B2rlcC;		// Port 2 capacitance (Farads)
	float		B1rlcF;		// Port 1 frequency
	float		B2rlcF;		// Port 2 frequency
} LNetBlockF;

typedef struct {
	uint8_t		B1RLCType;	// Port 1 hookup: S = series, P = parallel, T = trap
	uint8_t		B2RLCType;	// Port 2 hookup: S = series, P = parallel, T = trap
} LNetBlockG;

typedef struct {
	LNetBlockA	*pA;
	LNetBlockB	*pB;
	LNetBlockC	*pC;
	LNetBlockD	*pD;
	LNetBlockE	*pE;
	LNetBlockF	*pF;
	LNetBlockG	*pG;
} LNetBlockPtrs;

// Virtual Segments - used to avoid modeling wires just to insert a source
// or load on a transmission line.  Perhaps other uses as well.  EZNEC
// converts these to real wires far away for compatibility with older
// versions of the program.
//
// We don't really know at compile time how big the VSegNr array might
// be.  Fortunately, it is the last (and only) variable length field.
typedef struct {
	uint32_t	VWnr;		// Wire number to be replaced with the segment number
	uint32_t	VSegNr[1];	// Segment number
} VirtSegmentBlock;

typedef struct {
	BlkHeader	*pH;
	union {
		FreqSweepBlk		*pFSB;	// Single component
		WireInsBlock		*pWIB;	// Single component
		TLineLossBlock		*pTLLB;	// Single component
		TransformerBlockPtrs	TB;	// Multiple component
		LNetBlockPtrs		LNB;	// Multiple component
		VirtSegmentBlock	*pVSB;	// Single component
	} u;
} BLOCK;

typedef struct {
	RecType1	*pRec1;
	RecType2	**ppRec2;
	RecType3	*pRec3;
	RecType4	*pRec4;
	BLOCK		**ppBlks;
} POINTERS;

typedef struct {
	double		xyz;		// For converting endpoints
	double		wdiam;		// For converting wire diameters
	double		tldB;		// For converting transmission line dB
} CONVERSION_FACTORS;

// Globals
double			PI = 4.0 * atan2(1.0, 1.0);
void			*gIMap;			// Pointer to mmapped input file.
off_t			gInputFileSize;		// Size of input file in bytes.
int			gStartVarLenBlocks;	// Offset to the first varlen block.
POINTERS		gPointers;		// Pointers to all records and blocks.
CONVERSION_FACTORS	gConvert;		// Conversion factors
double			gFrequency;		// Frequency in MHz
int			gVarCount;		// Number of variable-length blocks
int			gNecVersion;		// NEC version (2, 4 or 5)
FreqSweepBlk		*gpFSB;			// Single component
WireInsBlock		*gpWIB;			// Single component
TLineLossBlock		*gpTLLB;		// Single component
TransformerBlockPtrs	gTB;			// Multiple component
LNetBlockPtrs		gLNB;			// Multiple component
VirtSegmentBlock	*gpVSB;			// Single component
uint32_t		gVirtualSegs[VSEG_MAX];	// Virtual segments
uint32_t		gVSegCount;		// Number of virtual segments
uint32_t		gVSegWire;		// Tag for all virtual wires
int			gSyntheticWire;		// Synthetic wires

// Copy a string, but always leave room for a null terminator, and add
// one if needed.
size_t
strlcpy(
		char *dst,
		const char *src,
		size_t maxlen
		)
{
	// Length including null terminator.
	const size_t srclen = strlen(src) + 1;

	if(srclen < maxlen) {
		// It fits with no truncation, including the null
		// terminator.
		memcpy(dst, src, srclen);
	} else if(maxlen != 0) {
		// We have to truncate.  Copy, leaving room for the
		// null terminator.
		memcpy(dst, src, maxlen - 1);

		// And add the null terminator.
		dst[maxlen - 1] = 0;
	}

	// This is the size of the string we tried to create, and
	// doesn't take the truncation into account.
	return srclen - 1;
}

// Some strings in .ez files have no null terminator, so we have
// to loop through the maximum string length instead.
void
printStr(char *p, int len)
{
	int i;

	for(i = 0; i < len; i++) {
		fputc(p[i], stderr);
	}
	fputc('\n', stderr);
}

typedef struct {
	double x;
	double y;
	double z;
} ENDPOINT;

typedef struct {
	ENDPOINT start;
	ENDPOINT end;
} ENDPOINTS;

int
findSegmentCoordinates(RecType2 *pWire, int segNo, ENDPOINTS *result)
{
	double deltaX;
	double deltaY;
	double deltaZ;

	double segLenX;
	double segLenY;
	double segLenZ;

	if(!result) {
		return -1;
	}

	if(segNo < 1 || segNo > pWire->WSegs) {
		return -1;
	}

	deltaX = pWire->WEnd2_X - pWire->WEnd1_X;
	deltaY = pWire->WEnd2_Y - pWire->WEnd1_Y;
	deltaZ = pWire->WEnd2_Z - pWire->WEnd1_Z;

	segLenX = deltaX / pWire->WSegs;
	segLenY = deltaY / pWire->WSegs;
	segLenZ = deltaZ / pWire->WSegs;
	
	result->start.x = pWire->WEnd1_X + segLenX * (segNo - 1);
	result->start.y = pWire->WEnd1_Y + segLenY * (segNo - 1);
	result->start.z = pWire->WEnd1_Z + segLenZ * (segNo - 1);
	
	result->end.x = pWire->WEnd1_X + segLenX * segNo;
	result->end.y = pWire->WEnd1_Y + segLenY * segNo;
	result->end.z = pWire->WEnd1_Z + segLenZ * segNo;

	return 0;
}

void
dumpRecType1(RecType1 *p)
{
	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump RecType1                                                            @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "RecType1: OldMaxMWSL = %d\n",	p->OldMaxMWSL);
	fprintf(stderr, "RecType1: NM = %d\n",		p->NM);
	fprintf(stderr, "RecType1: BdryType = %c\n",	p->BdryType);
	fprintf(stderr, "RecType1: FMHz = %.7g\n",	(double)p->FMHz);
	fprintf(stderr, "RecType1: OldPType = %d\n",	p->OldPType);
	fprintf(stderr, "RecType1: Pangle = %.7g\n",	(double)p->Pangle);
	fprintf(stderr, "RecType1: PStep = %.7g\n",	(double)p->PStep);
	fprintf(stderr, "RecType1: Title = ");		printStr(p->Title, TLEN);
	fprintf(stderr, "RecType1: OldNW = %d\n",	p->OldNW);
	fprintf(stderr, "RecType1: NSrc = %d\n",	p->NSrc);
	fprintf(stderr, "RecType1: NL = %d\n",		p->NL);
	fprintf(stderr, "RecType1: Gtype = %d\n",	p->Gtype);
	fprintf(stderr, "RecType1: NR = %d\n",		p->NR);
	fprintf(stderr, "RecType1: RDia = %.7g\n",	(double)p->RDia);
	fprintf(stderr, "RecType1: RDiaGa = %d\n",	p->RDiaGa);
	fprintf(stderr, "RecType1: Ck = %d\n",		p->Ck);
	fprintf(stderr, "RecType1: Units = %d\n",	p->Units);
	fprintf(stderr, "RecType1: PRange = %d\n",	p->PRange);
	fprintf(stderr, "RecType1: OldPPol = %d\n",	p->OldPPol);
	fprintf(stderr, "RecType1: ORdb = %.7g\n",	(double)p->ORdb);
	fprintf(stderr, "RecType1: PStart = %.7g\n",	(double)p->PStart);
	fprintf(stderr, "RecType1: PEnd = %.7g\n",	(double)p->PEnd);
	fprintf(stderr, "RecType1: OldLType = %d\n",	p->OldLType);
	fprintf(stderr, "RecType1: GWDist = %.7g\n",	(double)p->GWDist);
	fprintf(stderr, "RecType1: GWZ = %.7g\n",		(double)p->GWZ);
	fprintf(stderr, "RecType1: LNetType = %d\n",	p->LNetType);
	fprintf(stderr, "RecType1: AnalRes = %.7g\n",	(double)p->AnalRes);
	fprintf(stderr, "RecType1: RefdB = %.7g\n",	(double)p->RefdB);
	fprintf(stderr, "RecType1: WRho = %.7g\n",	(double)p->WRho);
	fprintf(stderr, "RecType1: WMu = %.7g\n",		(double)p->WMu);
	fprintf(stderr, "RecType1: NP = %d\n",		p->NP);
	fprintf(stderr, "RecType1: ArrayFiles = %d\n",	p->ArrayFiles);
	fprintf(stderr, "RecType1: SWRZ0 = %.7g\n",	(double)p->SWRZ0);
	fprintf(stderr, "RecType1: VerCode = %d\n",	p->VerCode);
	fprintf(stderr, "RecType1: DiaUnits = %d\n",	p->DiaUnits);
	fprintf(stderr, "RecType1: LType = %d\n",	p->LType);
	fprintf(stderr, "RecType1: PPol = %d\n",	p->PPol);
	fprintf(stderr, "RecType1: NT = %d\n",		p->NT);
	fprintf(stderr, "RecType1: PType = %d\n",	p->PType);
	fprintf(stderr, "RecType1: GAnal = %d\n",	p->GAnal);
	fprintf(stderr, "RecType1: MaxMWSL = %d\n",	p->MaxMWSL);
	fprintf(stderr, "RecType1: NW = %d\n",		p->NW);
	fprintf(stderr, "RecType1: dBSum = %.7g\n",	(double)p->dBSum);
	fprintf(stderr, "RecType1: PStep3D = %.7g\n",	(double)p->PStep3D);
	fprintf(stderr, "RecType1: MiscFlags = %d\n",	p->MiscFlags);
	fprintf(stderr, "RecType1: PgmVerCode = %d\n",	p->PgmVerCode);
	fprintf(stderr, "RecType1: LinRange2D = %.7g\n",	(double)p->LinRange2D);
	fprintf(stderr, "RecType1: NX = %d\n",		p->NX);
	fprintf(stderr, "RecType1: NN = %d\n",		p->NN);
	fprintf(stderr, "RecType1: NLNet = %d\n",	p->NLNet);
	fprintf(stderr, "RecType1: NVirt = %d\n",	p->NVirt);
	fprintf(stderr, "RecType1: Reserved1 = %d\n",	p->Reserved1);
	fprintf(stderr, "RecType1: HAGndType = %d\n",	p->HAGndType);
	//fprintf(stderr, "RecType1: Reserved[7] = %d\n", p->Reserved[7]);

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpRecType2(int idx, RecType2 *p)
{
	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump RecType2 #%d                                                        @@\n", idx); 
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "RecType2: MSigma = %.7g\n",	(double)p->MSigma);
	fprintf(stderr, "RecType2: MEps = %.7g\n",	(double)p->MEps);
	fprintf(stderr, "RecType2: MCoord = %.7g\n",	(double)p->MCoord);
	fprintf(stderr, "RecType2: MHt = %.7g\n",		(double)p->MHt);
	fprintf(stderr, "RecType2: WEnd1_X = %.7g\n",	(double)p->WEnd1_X);
	fprintf(stderr, "RecType2: WEnd1_Y = %.7g\n",	(double)p->WEnd1_Y);
	fprintf(stderr, "RecType2: WEnd1_Z = %.7g\n",	(double)p->WEnd1_Z);
	fprintf(stderr, "RecType2: WEnd2_X = %.7g\n",	(double)p->WEnd2_X);
	fprintf(stderr, "RecType2: WEnd2_Y = %.7g\n",	(double)p->WEnd2_Y);
	fprintf(stderr, "RecType2: WEnd2_Z = %.7g\n",	(double)p->WEnd2_Z);
	fprintf(stderr, "RecType2: WireDia = %.7g\n",	(double)p->WireDia);
	fprintf(stderr, "RecType2: WDiaGa = %d\n",	p->WDiaGa);
	fprintf(stderr, "RecType2: WSegs = %d\n",	p->WSegs);
	fprintf(stderr, "RecType2: SWNr = %d\n",	p->SWNr);
	fprintf(stderr, "RecType2: SWPct = %.7g\n",	(double)p->SWPct);
	fprintf(stderr, "RecType2: SMP_M = %.7g\n",	(double)p->SMP_M);
	fprintf(stderr, "RecType2: SMP_Pdeg = %.7g\n",	(double)p->SMP_Pdeg);
	fprintf(stderr, "RecType2: Stype = %d\n",	p->Stype);
	fprintf(stderr, "RecType2: LWNr = %d\n",	p->LWNr);
	fprintf(stderr, "RecType2: LWPct = %.7g\n",	(double)p->LWPct);
	fprintf(stderr, "RecType2: LZ_R = %.7g\n",	(double)p->LZ_R);
	fprintf(stderr, "RecType2: LZ_I = %.7g\n",	(double)p->LZ_I);
	fprintf(stderr, "RecType2: LNum_S0 = %.7g\n",	(double)p->LNum_S0);
	fprintf(stderr, "RecType2: LNum_S1 = %.7g\n",	(double)p->LNum_S1);
	fprintf(stderr, "RecType2: LNum_S2 = %.7g\n",	(double)p->LNum_S2);
	fprintf(stderr, "RecType2: LNum_S3 = %.7g\n",	(double)p->LNum_S3);
	fprintf(stderr, "RecType2: LNum_S4 = %.7g\n",	(double)p->LNum_S4);
	fprintf(stderr, "RecType2: LNum_S5 = %.7g\n",	(double)p->LNum_S5);
	fprintf(stderr, "RecType2: LDen_S0 = %.7g\n",	(double)p->LDen_S0);
	fprintf(stderr, "RecType2: LDen_S1 = %.7g\n",	(double)p->LDen_S1);
	fprintf(stderr, "RecType2: LDen_S2 = %.7g\n",	(double)p->LDen_S2);
	fprintf(stderr, "RecType2: LDen_S3 = %.7g\n",	(double)p->LDen_S3);
	fprintf(stderr, "RecType2: LDen_S4 = %.7g\n",	(double)p->LDen_S4);
	fprintf(stderr, "RecType2: LDen_S5 = %.7g\n",	(double)p->LDen_S5);
	fprintf(stderr, "RecType2: TLWNr1 = %d\n",	p->TLWNr1);
	fprintf(stderr, "RecType2: TLWPct1 = %.7g\n",	(double)p->TLWPct1);
	fprintf(stderr, "RecType2: TLWNr2 = %d\n",	p->TLWNr2);
	fprintf(stderr, "RecType2: TLWPct2 = %.7g\n",	(double)p->TLWPct2);
	fprintf(stderr, "RecType2: TLZ0 = %.7g\n",	(double)p->TLZ0);
	fprintf(stderr, "RecType2: TLLen = %.7g\n",	(double)p->TLLen);
	fprintf(stderr, "RecType2: TLVF = %.7g\n",	(double)p->TLVF);
	fprintf(stderr, "RecType2: LConn = %d\n",	p->LConn);
	fprintf(stderr, "RecType2: strRLCType = %d\n",	p->strRLCType);
	fprintf(stderr, "RecType2: sngR = %.7g\n",	(double)p->sngR);
	fprintf(stderr, "RecType2: sngL = %.7g\n",	(double)p->sngL);
	fprintf(stderr, "RecType2: sngC = %.7g\n",	(double)p->sngC);
	fprintf(stderr, "RecType2: sngRFreqMHz = %.7g\n",	(double)p->sngRFreqMHz);
	//fprintf(stderr, "RecType2: Reserved[3] = %.7g\n", (double)p->Reserved[3]);

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpRecType3(RecType3 *p)
{
	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump RecType3                                                            @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "RecType3: NFType = %d\n", p->NFType);
	fprintf(stderr, "RecType3: NFCoordType = %d\n",	 p->NFCoordType);
	fprintf(stderr, "RecType3: NF1Start = %.7g\n",	(double)p->NF1Start);
	fprintf(stderr, "RecType3: NF1Stop = %.7g\n",	(double)p->NF1Stop);
	fprintf(stderr, "RecType3: NF1Step = %.7g\n",	(double)p->NF1Step);
	fprintf(stderr, "RecType3: NF2Start = %.7g\n",	(double)p->NF2Start);
	fprintf(stderr, "RecType3: NF2Stop = %.7g\n",	(double)p->NF2Stop);
	fprintf(stderr, "RecType3: NF2Step = %.7g\n",	(double)p->NF2Step);
	fprintf(stderr, "RecType3: NF3Start = %.7g\n",	(double)p->NF3Start);
	fprintf(stderr, "RecType3: NF3Stop = %.7g\n",	(double)p->NF3Stop);
	fprintf(stderr, "RecType3: NF3Step = %.7g\n",	(double)p->NF3Step);
	//fprintf(stderr, "RecType3: Reserved[132] = %.7g\n", (double)p->Reserved[132]);

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpBlkHeader(BlkHeader *p)
{
	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump BlkHeader                                                           @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "BlkHeader: BlockType = %d\n",	p->BlockType);
	fprintf(stderr, "BlkHeader: BlockLen = %d\n",	p->BlockLen);
	fprintf(stderr, "BlkHeader: BlockRev = %d\n",	p->BlockRev);

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpFreqSweepBlock(BlkHeader *pH, FreqSweepBlk *p)
{
	if(!gDebug) {
		return;
	}

	int bytes_remaining;
	int i;
	uint16_t *pEndTest;
	char *pFN;

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump Frequency Sweep Block                                               @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	// Everything up to FreqInFile is present.  Start with them.
	fprintf(stderr, "FreqSweepBlk: DataLen = %d\n",	p->DataLen);
	fprintf(stderr, "FreqSweepBlk: FIFLen = %d\n",	p->FIFLen);
	fprintf(stderr, "FreqSweepBlk: PPFLen = %d\n",	p->PPFLen);
	fprintf(stderr, "FreqSweepBlk: SCPFLen = %d\n",	p->SCPFLen);
	fprintf(stderr, "FreqSweepBlk: DFLen = %d\n",	p->DFLen);
	fprintf(stderr, "FreqSweepBlk: AFLen = %d\n",	p->AFLen);
	fprintf(stderr, "FreqSweepBlk: Flags = %d\n",	p->Flags);
	fprintf(stderr, "FreqSweepBlk: FStart = %.7g\n",	(double)p->FStart);
	fprintf(stderr, "FreqSweepBlk: FStop = %.7g\n",	(double)p->FStop);
	fprintf(stderr, "FreqSweepBlk: FStep = %.7g\n",	(double)p->FStep);

	// Determine how many bytes are left in the block, after we remove
	// the header and the above fields.
       	bytes_remaining = pH->BlockLen - sizeof(BlkHeader) - offsetof(FreqSweepBlk, FreqInFile);

	if(bytes_remaining >= sizeof(uint16_t)) {
		// There is one "guaranteed present" uint16_t field after the
		// various "file" arrays (some or all of which may be zero
		// length).  We test for (bytes_remaining >= sizeof(uint16_t))
		// because the uint16_t field must be present.
		//
		// Thus, (bytes_remaining == sizeof(uint16_t)) means that just
		// EndTest is present.  Anything bigger means one or more
		// non-empty "file" arrays are present too.
		//
		// We have to separate the various "file" arrays based on the
		// Len parameters.
		//
		// We will trust the various Len parameters to walk through
		// the array, but we first check the EndTest.  If it is wrong,
		// then we have a problem.
		//
		// Since FreqInFile comes first, we key off it.
		pEndTest = (uint16_t *)(p->FreqInFile + bytes_remaining - sizeof(uint16_t));
		if(*pEndTest == 12345) {
			pFN = p->FreqInFile;

			fprintf(stderr, "FreqSweepBlk: Frequency File = \"");
			for(i = 0; i < p->FIFLen; i++) {
				fprintf(stderr, "%c", *pFN++);
			}
			fprintf(stderr, "\"\n");

			fprintf(stderr, "FreqSweepBlk: Pattern File = \"");
			for(i = 0; i < p->PPFLen; i++) {
				fprintf(stderr, "%c", *pFN++);
			}
			fprintf(stderr, "\"\n");

			fprintf(stderr, "FreqSweepBlk: Smith File = \"");
			for(i = 0; i < p->SCPFLen; i++) {
				fprintf(stderr, "%c", *pFN++);
			}
			fprintf(stderr, "\"\n");

			fprintf(stderr, "FreqSweepBlk: Data File = \"");
			for(i = 0; i < p->DFLen; i++) {
				fprintf(stderr, "%c", *pFN++);
			}
			fprintf(stderr, "\"\n");

			fprintf(stderr, "FreqSweepBlk: Reserved File = \"");
			for(i = 0; i < p->AFLen; i++) {
				fprintf(stderr, "%c", *pFN++);
			}
			fprintf(stderr, "\"\n");

			pEndTest = (uint16_t *)pFN;
			fprintf(stderr, "FreqSweepBlk: EndTest = %d, as expected\n", *pEndTest);
		} else {
			fprintf(stderr, "FreqSweepBlk: EndTest = %d, but expected 12345\n", *pEndTest);
		}
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpWireInsBlock(BlkHeader *pH, WireInsBlock *p)
{
	int i;

	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump Wire Insulation Block                                               @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "WireInsBlock: NumWires = %d\n", p->NumWires);
	for(i = 0; i < p->NumWires; i++) {
		fprintf(stderr, "WireInsBlock: DielC[%d] = %.7g\n",	i + 1, p->Wires[i].DielC);
		fprintf(stderr, "WireInsBlock: Thk[%d] = %.7g\n",		i + 1, p->Wires[i].Thk);
		fprintf(stderr, "WireInsBlock: LTan[%d] = %.7g\n",	i + 1, p->Wires[i].LTan);
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpTLineLossBlock(BlkHeader *pH, TLineLossBlock *p)
{
	int i;
	float *pFloat;

	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump Transmission Line Loss Block                                        @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "TLineLossBlock: NumLines = %d\n", p->NumLines);

	pFloat = &p->Loss[0];
	for(i = 0; i < p->NumLines; i++) {
		fprintf(stderr, "TLineLossBlock: Loss[%d] = %.7g\n", i + 1, *pFloat++);
	}
	for(i = 0; i < p->NumLines; i++) {
		fprintf(stderr, "TLineLossBlock: LossFreq[%d] = %.7g\n", i + 1, *pFloat++);
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpTransformerBlock(BlkHeader *pH, TransformerBlockPtrs *p)
{
	int i;

	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump Transformer Block                                                   @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "TransformerBlock: NX = %d\n", p->pA->NX);

	for(i = 0; i <= p->pA->NX; i++) {
		fprintf(stderr, "TransformerBlock: Port1 Tx[%d] Wire = %d\n", i, p->pB[i].P1WNr);
		fprintf(stderr, "TransformerBlock: Port2 Tx[%d] Wire = %d\n", i, p->pB[i].P2WNr);
	}

	for(i = 0; i <= p->pA->NX; i++) {
		fprintf(stderr, "TransformerBlock: Port1 Tx[%d] Percent = %.7g\n", i, p->pC[i].P1WPct);
		fprintf(stderr, "TransformerBlock: Port2 Tx[%d] Percent = %.7g\n", i, p->pC[i].P2WPct);
	}

	for(i = 0; i <= p->pA->NX; i++) {
		fprintf(stderr, "TransformerBlock: Port1 Tx[%d] Z = %.7g\n", i, p->pD[i].P1RelZ);
		if(p->pD[i].P2RelZ >= 0) {
			fprintf(stderr, "TransformerBlock: Port2 Tx[%d] Z = %.7g Norm\n", i, p->pD[i].P2RelZ);
		} else {
			fprintf(stderr, "TransformerBlock: Port2 Tx[%d] Z = %.7g Rev\n", i, -p->pD[i].P2RelZ);
		}
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpLNetBlock(BlkHeader *pH, LNetBlockPtrs *p)
{
	int i;

	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump L-Network Block                                                     @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "\n");

	fprintf(stderr, "LNetBlock: NL = %d\n", p->pA->NL);

	for(i = 0; i <= p->pA->NL; i++) {
		fprintf(stderr, "LNetBlock: Port1 Tx[%d] Wire = %d\n", i, p->pB[i].P1WNr);
		fprintf(stderr, "LNetBlock: Port2 Tx[%d] Wire = %d\n", i, p->pB[i].P2WNr);
	}

	for(i = 0; i <= p->pA->NL; i++) {
		fprintf(stderr, "LNetBlock: Port1 Tx[%d] Percent = %.7g\n", i, p->pC[i].P1WPct);
		fprintf(stderr, "LNetBlock: Port2 Tx[%d] Percent = %.7g\n", i, p->pC[i].P2WPct);
	}

	for(i = 0; i <= p->pA->NL; i++) {
		fprintf(stderr, "LNetBlock: Port1 Tx[%d] R = %.7g\n", i, p->pD[i].B1R);
		fprintf(stderr, "LNetBlock: Port1 Tx[%d] X = %.7g\n", i, p->pD[i].B1X);
		fprintf(stderr, "LNetBlock: Port2 Tx[%d] R = %.7g\n", i, p->pD[i].B2R);
		fprintf(stderr, "LNetBlock: Port2 Tx[%d] X = %.7g\n", i, p->pD[i].B2X);
	}

	fprintf(stderr, "LNetBlock: NrRLC = %d\n", p->pE->NrRLC);

	if(p->pE->NrRLC) {
		for(i = 0; i <= p->pA->NL; i++) {
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] R = %.7g\n", i, p->pF[i].B1rlcR);
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] L = %.7g\n", i, p->pF[i].B1rlcL);
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] C = %.7g\n", i, p->pF[i].B1rlcC);
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] F = %.7g\n", i, p->pF[i].B1rlcF);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] R = %.7g\n", i, p->pF[i].B2rlcR);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] L = %.7g\n", i, p->pF[i].B2rlcL);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] C = %.7g\n", i, p->pF[i].B2rlcC);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] F = %.7g\n", i, p->pF[i].B2rlcF);
		}

		for(i = 0; i <= p->pA->NL; i++) {
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] Type = %d\n", i, p->pG[i].B1RLCType);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] Type = %d\n", i, p->pG[i].B2RLCType);
		}
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpVirtSegmentBlock(BlkHeader *pH, VirtSegmentBlock *p)
{
	int i;

	if(!gDebug) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump Virtual Segment Block                                               @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	// Virtual wire number.
	fprintf(stderr, "VWnr = %d\n", p->VWnr);

	// The item at 0 is a dummy, but print it anyway.
	for(i = 0; i <= gVSegCount; i++) {
		fprintf(stderr, "VSegNr[%d] = %d\n", i, p->VSegNr[i]);
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

// Convert "percent along a wire" to a segment number.
int
virtualIndex(int segments, double percent)
{
	int idx = (int)round(((percent / 100.0) + (1.0 / (segments * 2.0))) * segments);

	// We shouldn't need this but there is something strange about how
	// gcc is doing math as compared to visual basic.
	if(idx > segments) {
		if(gDebug) fprintf(stderr, "virtualIndex of %.7g is %d (overflow %d)\n", percent, segments, idx);
		return segments;
	}

	if(gDebug) fprintf(stderr, "virtualIndex of %.7g is %d\n", percent, idx);
	return idx;
}

// Use mmap to get access to the EZ file.
int
mmapInput(char *pName)
{
	int		rv;
	int		fd;
	struct stat	sb;

	// Get the file size, as we need that for mmap();
	errno = 0;
	if((rv = stat(pName, &sb)) < 0) {
		fprintf(stderr, "Cannot stat %s, got %d, errno %d\n", pName, rv, errno);
		return -1;
	}
	gInputFileSize = sb.st_size;
	if(gDebug) fprintf(stderr, "length of file 0x%x\n", gInputFileSize);

	// Open the EZ file.  We open it read/write, because we need the
	// ability to modify mmapped variables.  But we don't actually want
	// to affect the file, so we will use a private mapping.
	errno = 0;
	if((fd = open(pName, O_RDWR)) < 0) {
		fprintf(stderr, "Cannot open %s, got %d, errno %d\n", pName, fd, errno);
		return -1;
	}

	// Map it into our process space.  Allow reading and writing, but use
	// a private (copy-on-write) map so we don't change the underlying
	// file.
	errno = 0;
	if((gIMap = mmap(NULL, gInputFileSize, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0)) == MAP_FAILED) {
		close(fd);
		fprintf(stderr, "Cannot mmap %s, got %d, errno %d\n", pName, fd, errno);
		return -1;
	}

	// We don't need the file descriptor once the map has been made.
	close(fd);
	return 0;
}

// Figure out what units we should use.
void
setConversionFactors()
{
	// Apparently, there are some files where the units are not set
	// properly.  In that case, we force "Wavelength".
	if(gPointers.pRec1->Units == 0) {
		gPointers.pRec1->Units = 'W';
	}
	if(gPointers.pRec1->DiaUnits == 0) {
		gPointers.pRec1->DiaUnits = 'W';
	}

	switch(gPointers.pRec1->Units) {
		case 'M':	// Use mm for wire diameter, meters for everything else 
			gConvert.xyz = 1;
			gConvert.wdiam = 0.001;
			gConvert.tldB = 1;
			break;
		case 'L':	// Use meters for TL Loss, mm for everything else
			gConvert.xyz = 0.001;
			gConvert.wdiam = 0.001;
			gConvert.tldB = 1;
			break;
		case 'F':	// Use inches for wire diameter, feet for everything else
			gConvert.xyz = 0.3048;
			gConvert.wdiam = 0.0254;
			gConvert.tldB = 0.3048;
			break;
		case 'I':	// Use feet for TL Loss, inches for everything else
			gConvert.xyz = 0.0254;
			gConvert.wdiam = 0.0254;
			gConvert.tldB = 0.3048;
			break;
		case 'W':	// Wavelength has to be converted based on frequency
			gConvert.xyz = 299.792458 / gFrequency;	// Use meters for wire ends

			switch(gPointers.pRec1->DiaUnits) {
				case 'L':	// Use mm for wire diameter
					gConvert.wdiam = 0.001;
					break;
				case 'I':	// Use inches for wire diameter
					gConvert.wdiam = 0.0254;
					break;
				case 'W':	// Use wavelength for wire diameter
					gConvert.wdiam = gConvert.xyz;
					break;
			}

			gConvert.tldB = 0.3048; // Use feet for TL Loss
	}
}

// Fill in pointer for the RecType1 block and do a little setup.
int
mapRecType1()
{
	// Map the global header (RecType1 block).  It starts at byte 0.
	MAP(gPointers.pRec1, RecType1, gIMap, 0);

	if(gPointers.pRec1->MaxMWSL == 0) {
		// OldMaxMWSL is 16 bit, and has been replaced by MaxMWSL
		// which is 32 bit.  Older programs may not have filled
		// MaxMWSL in, I suppose.
		//
		// But I have no way to test that.
		gPointers.pRec1->MaxMWSL = gPointers.pRec1->OldMaxMWSL;
	}

	if(gPointers.pRec1->NW == 0) {
		// OldNW is 16 bit, and has been replaced by NW
		// which is 32 bit.  Older programs may not have filled
		// NW in, I suppose.
		//
		// But I have no way to test that.
		gPointers.pRec1->NW = gPointers.pRec1->OldNW;
	}

	// Very rudimentary sanity check.
	if(gPointers.pRec1->Ck != 1304) {
		fprintf(stderr, "pRec1->Ck != 1304\n");
		return -1;
	}

	// Lots of things depend on the frequency, so set a global for
	// convenience.
	gFrequency = gPointers.pRec1->FMHz;

	// We need to know about virtual segments early.
	gVSegCount = gPointers.pRec1->NVirt;

	// Set up the conversion factors.  All values in the file use meters,
	// but we want to be able to convert to the user's preferred units.
	setConversionFactors();

	dumpRecType1(gPointers.pRec1);
	return 0;
}

// Fill in pointers for all the RecType2 blocks.
int
mapRecType2()
{
	int			i;
	int			position;

	// There is at least one RecType2, but probably a lot more.
	// We need an array of pointers for them.
	if((gPointers.ppRec2 = malloc(gPointers.pRec1->MaxMWSL * sizeof(RecType2 *))) == NULL) {
		fprintf(stderr, "No space to map RecType2 records\n");
		return -1;
	}

	// Map the RecType2 record(s).  They start immediately after the
	// global record.
	position = PRIMARY_BS;
	for(i = 0; i < gPointers.pRec1->MaxMWSL; i++) {
		MAP(gPointers.ppRec2[i], RecType2, gIMap, position);

		dumpRecType2(i, gPointers.ppRec2[i]);

		// Move to the next block.
		position += PRIMARY_BS;
	}

	return 0;
}

// Fill in pointer for the RecType3 block.
int
mapRecType3()
{
	int			position;

	// Map the near field (RecType3 block).  It starts after the
	// last RecType2 block.
	position = PRIMARY_BS + (gPointers.pRec1->MaxMWSL * PRIMARY_BS);
	MAP(gPointers.pRec3, RecType3, gIMap, position);
	dumpRecType3(gPointers.pRec3);
	return 0;
}

// Fill in pointers for all the variable-length blocks.
int
mapVarBlocks()
{
	int varStart;
	uint32_t varPosition;
	int i;
	BlkHeader *pH;
	BLOCK *pB;
	uint32_t start;
	uint32_t remain;
	uint32_t need;
	int foundFS = 0;
	int foundWI = 0;
	int foundTL = 0;
	int foundTX = 0;
	int foundLN = 0;
	int foundVS = 0;

	// There is always a single gPointers.pRec1 - that's the first
	// PRIMARY_BS in the varStart variable.
	//
	// Then there are a bunch of pRec2, also PRIMARY_BS bytes each.  NW,
	// NSrc, NL, NT, NM all ride in the same set of pRec2 blocks - that
	// is the parenthesized term.
	//
	// Then comes pRec3 (second to last PRIMARY_BS), and finally comes
	// pRec4 (the last PRIMARY_BS).
	varStart = PRIMARY_BS + (gPointers.pRec1->MaxMWSL * PRIMARY_BS) + PRIMARY_BS + PRIMARY_BS;

	// We don't know how many var blocks there are, so we first have to
	// scan through the file to get a count.  Then we can allocate pointer
	// space.
	gVarCount = 0;
	varPosition = varStart;
	while(varPosition < gInputFileSize) {
		MAP(pH, BlkHeader, gIMap, varPosition);
		gVarCount++;
		varPosition += pH->BlockLen;

		dumpBlkHeader(pH);
	}
	if(gDebug) fprintf(stderr, "gVarCount = %d, varPosition = %d, gInputFileSize = %d\n",
			gVarCount, varPosition, gInputFileSize);

	// Create space for the BLOCK pointers.
	if((gPointers.ppBlks = malloc(gVarCount * sizeof(BLOCK *))) == NULL) {
		fprintf(stderr, "No space to map varblock pointers\n");
		return -1;
	}

	// Create the BLOCK structures themselves.
	for(i = 0; i < gVarCount; i++) {
		if((gPointers.ppBlks[i] = malloc(sizeof(BLOCK))) == NULL) {
			fprintf(stderr, "No space to map varblocks\n");
			return -1;
		}
	}

	// Map the components.
	varPosition = varStart;
	for(i = 0; i < gVarCount; i++) {
		MAP(pH, BlkHeader, gIMap, varPosition);
		start = varPosition + sizeof(BlkHeader);
		remain = pH->BlockLen - sizeof(BlkHeader);

		pB = gPointers.ppBlks[i];
		pB->pH = pH;
		switch(pH->BlockType) {
			case VB_FREQ_SWEEP:
				if(foundFS) {
					fprintf(stderr, "Duplicate VB_FREQ_SWEEP - replacing!\n");
				}
				++foundFS;

				// Variable length - map the whole block.
				VMAP(pB->u.pFSB, FreqSweepBlk, gIMap, start, pH->BlockLen - sizeof(BlkHeader));
				gpFSB = pB->u.pFSB;
				dumpFreqSweepBlock(pH, gpFSB);
				break;

			case VB_WIRE_INS:
				if(foundWI) {
					fprintf(stderr, "Duplicate VB_WIRE_INS - replacing!\n");
				}
				++foundWI;

				// Variable length - map the whole block.
				VMAP(pB->u.pWIB, WireInsBlock, gIMap, start, pH->BlockLen - sizeof(BlkHeader));
				gpWIB = pB->u.pWIB;
				dumpWireInsBlock(pH, gpWIB);
				break;

			case VB_TL_LOSS:
				if(foundTL) {
					fprintf(stderr, "Duplicate VB_TL_LOSS - replacing!\n");
				}
				++foundTL;

				// Variable length - map the whole block.
				VMAP(pB->u.pTLLB, TLineLossBlock, gIMap, start, pH->BlockLen - sizeof(BlkHeader));
				gpTLLB = pB->u.pTLLB;
				dumpTLineLossBlock(pH, gpTLLB);
				break;

			case VB_TRANSFORMER:
				if(foundTX) {
					fprintf(stderr, "Duplicate VB_TRANSFORMER - replacing!\n");
				}
				++foundTX;

				// The A component is fixed length.
				need = sizeof(TransformerBlockA);
				VMAP(pB->u.TB.pA, TransformerBlockA, gIMap, start, need);
				start += need;
				remain -= need;

				// The B, C, and D components are variable
				// length.  Calculate the sizes.  There is
				// one extra (dummy/reserved) block at the
				// start of each, hence the "1 +".
				need = (1 + pB->u.TB.pA->NX) * sizeof(TransformerBlockB);
				VMAP(pB->u.TB.pB, TransformerBlockB, gIMap, start, need);
				start += need;
				remain -= need;

				need = (1 + pB->u.TB.pA->NX) * sizeof(TransformerBlockC);
				VMAP(pB->u.TB.pC, TransformerBlockC, gIMap, start, need);
				start += need;
				remain -= need;

				need = (1 + pB->u.TB.pA->NX) * sizeof(TransformerBlockD);
				VMAP(pB->u.TB.pD, TransformerBlockD, gIMap, start, need);
				start += need;
				remain -= need;

				// Structure copy.
				gTB = pB->u.TB;

				dumpTransformerBlock(pH, &gTB);
				break;

			case VB_L_NETWORK:
				if(foundLN) {
					fprintf(stderr, "Duplicate VB_L_NETWORK - replacing!\n");
				}
				++foundLN;
				// The A component is fixed length.
				need = sizeof(LNetBlockA);
				VMAP(pB->u.LNB.pA, LNetBlockA, gIMap, start, need);
				start += need;
				remain -= need;

				// The B, C, and D components are variable
				// length.  Calculate the sizes.  There is
				// one extra (dummy/reserved) block at the
				// start of each, hence the "1 +".
				need = (1 + pB->u.LNB.pA->NL) * sizeof(LNetBlockB);
				VMAP(pB->u.LNB.pB, LNetBlockB, gIMap, start, need);
				start += need;
				remain -= need;

				need = (1 + pB->u.LNB.pA->NL) * sizeof(LNetBlockC);
				VMAP(pB->u.LNB.pC, LNetBlockC, gIMap, start, need);
				start += need;
				remain -= need;

				need = (1 + pB->u.LNB.pA->NL) * sizeof(LNetBlockD);
				VMAP(pB->u.LNB.pD, LNetBlockD, gIMap, start, need);
				start += need;
				remain -= need;

				// The E component is fixed length.
				need = sizeof(LNetBlockE);
				VMAP(pB->u.LNB.pE, LNetBlockE, gIMap, start, need);
				start += need;
				remain -= need;

				if(pB->u.LNB.pE->NrRLC) {
					// These last two blocks are optional.

					// The F and D components are variable
					// length.  Calculate the sizes.  There is
					// one extra (dummy/reserved) block at the
					// start of each, hence the "1 +".
					need = (1 + pB->u.LNB.pA->NL) * sizeof(LNetBlockF);
					VMAP(pB->u.LNB.pF, LNetBlockF, gIMap, start, need);
					start += need;
					remain -= need;

					// There is one extra (dummy/reserved) block at the start,
					// hence the "1 +".
					need = (1 + pB->u.LNB.pA->NL) * sizeof(LNetBlockG);
					VMAP(pB->u.LNB.pG, LNetBlockG, gIMap, start, need);
					start += need;
					remain -= need;
				} else {
					pB->u.LNB.pF = 0;
					pB->u.LNB.pG = 0;
				}

				// Structure copy.
				gLNB = pB->u.LNB;

				dumpLNetBlock(pH, &gLNB);
				break;

			case VB_VIRTUAL_SEG:
				if(foundVS) {
					fprintf(stderr, "Duplicate VB_VIRTUAL_SEG - replacing!\n");
				}
				++foundVS;

				// Variable length - map the whole block.
				VMAP(pB->u.pVSB, VirtSegmentBlock, gIMap, start, pH->BlockLen - sizeof(BlkHeader));
				gpVSB = pB->u.pVSB;
				dumpVirtSegmentBlock(pH, gpVSB);
				break;

			case VB_Y_PARAM:
			case VB_PLANE_WAVE_SRC:
			default:
				// Ignore - we don't know what this is.
				if(gDebug) fprintf(stderr, "Unhandled block of type %d\n", pH->BlockType);
				break;
		}

		varPosition += pH->BlockLen;
	}
}

void
loadVirtualWires()
{
	if(gpVSB) {
		int i;

		gVSegWire = gpVSB->VWnr;

		for(i = 0; i < gVSegCount; i++) {
			gVirtualSegs[i] = gpVSB->VSegNr[i];
		}
	} else {
		gVSegWire = -1;
	}
}

// Print CM and CE cards.
void
printCMCE(FILE *pOut)
{
	char buf[TLEN + 1];
	char *p;
	char *q;
	int i;

	// String has a fixed length and no null terminator.
	memcpy(buf, gPointers.pRec1->Title, TLEN);
	buf[TLEN] = 0;

	// Trim leading spaces, if any.
	for(p = buf; *p != 0; p++) {
		if(*p != ' ') {
			break;
		}
	}

	// Trim trailing spaces, if any.
	for(q = p + strlen(p); q != p; q--) {
		if(*q != 0 && *q != ' ') {
			break;
		}
		*q = 0;
	}

	fprintf(pOut, "CM %s\n", p);
	fprintf(pOut, "CE\n");
}

// Print GW cards.
void
printWires(FILE *pOut)
{
	int i;
	RecType2 *pWire;
	RecType2 *pTL;
	double tmp;
	uint16_t wireNo1;	// Number of the wire side-1 connects to.
	uint16_t wireNo2;	// Number of the wire side-2 connects to.
	int segNo;		// Segment number.
	float percent;		// Percentage along wire.
	ENDPOINTS points;	// Endpoints of a segment.

	// We might want to handle virtual wires by finding #18 block first.
	for(i = 0; i < gPointers.pRec1->NW; i++) {
		pWire = gPointers.ppRec2[i];

		fprintf(pOut, "GW %d,%d,", i + 1, pWire->WSegs);

		fprintf(pOut, "%.7g,", pWire->WEnd1_X / gConvert.xyz);
		fprintf(pOut, "%.7g,", pWire->WEnd1_Y / gConvert.xyz);
		fprintf(pOut, "%.7g,", pWire->WEnd1_Z / gConvert.xyz);

		fprintf(pOut, "%.7g,", pWire->WEnd2_X / gConvert.xyz);
		fprintf(pOut, "%.7g,", pWire->WEnd2_Y / gConvert.xyz);
		fprintf(pOut, "%.7g,", pWire->WEnd2_Z / gConvert.xyz);

		if(pWire->WDiaGa == (uint16_t)-1) {
			// There is only one scale factor in the GS card, so
			// we cannot show wire diameter in different units
			// than wire endpoints.
			//
			// NEC wants the radius, hence the divide-by-2.
			fprintf(pOut, "%.7g\n", (pWire->WireDia / 2.0) / gConvert.xyz);
		} else {
			// Calculate the wire diameter in meters, based on AWG
			// algorithm.  Works for wire GA 0 and thinner.
			//
			// Print it based on the user's scalefactor.
			//
			// NEC wants the radius, hence the divide-by-2.
			tmp = 0.005 * pow(92.0, ((36.0 - pWire->WDiaGa) / 39.0)) * 0.0254;
			fprintf(pOut, "%.7g\n", (tmp / 2.0) / gConvert.xyz);
		}
	}

	// We may need to synthesize extra wires for open or shorted
	// transmission lines at this point.
	//
	// Start off synthetic wires at a safe number.  We'll have to
	// use the same starting value when we process TLs again later.
	gSyntheticWire = WIRE_MAX + 1;
	for(i = 0; i < gPointers.pRec1->NT; i++) {
		pTL = gPointers.ppRec2[i];

		wireNo1 = pTL->TLWNr1;
		wireNo2 = pTL->TLWNr2;
		if(wireNo1 != SHORTED_END && wireNo1 != OPEN_END && wireNo2 != SHORTED_END && wireNo2 != OPEN_END) {
			// This is a nice normal transmission line
			// connected to two actual wires.  There is
			// no need to synthesize a wire.
			continue;
		}

		if((wireNo1 == SHORTED_END || wireNo1 == OPEN_END) && (wireNo2 == SHORTED_END || wireNo2 == OPEN_END)) {
			// This is bizarre - both ends of the TL are virtual.
			// Ignore it.  We'll also ignore it later, when we
			// emit the TL cards.
			continue;
		}

		// Exactly one end of the TL is virtual, so we have to
		// synthesize one wire.  Get the other end of the TL
		// so we can use it as a model for the synthesized wire.
		if(wireNo1 == SHORTED_END || wireNo1 == OPEN_END) {
			// Wire 1 is virtual, so wire 2 is real.
			pWire = gPointers.ppRec2[wireNo2 - 1];
			percent = pTL->TLWPct2;
		} else {
			// Wire 2 is virtual, so wire 1 is real.
			pWire = gPointers.ppRec2[wireNo1 - 1];
			percent = pTL->TLWPct1;
		}

		// Based on the percentage, figure out which segment the
		// TL connects to on the real wire, and find its xyz
		// coordinates.
		//
		// We will create a new wire corresponding to that segment's
		// xyz coordinates, and offset by the length of the TL in Z.
		segNo = virtualIndex(pWire->WSegs, percent);
		findSegmentCoordinates(pWire, segNo, &points);

		// We always create a single-segment wire.
		fprintf(pOut, "GW %d,%d,", gSyntheticWire++, 1);

		// Offset it in negative Z by the length of the TL.
		//
		// FIXME:  We might have to do something more complicated
		// here.  Perhaps we need to offset so as not to collide
		// with other wires?
		fprintf(pOut, "%.7g,", points.start.x / gConvert.xyz);
		fprintf(pOut, "%.7g,", points.start.y / gConvert.xyz);
		fprintf(pOut, "%.7g,", (points.start.z - pTL->TLLen) / gConvert.xyz);

		fprintf(pOut, "%.7g,", points.end.x / gConvert.xyz);
		fprintf(pOut, "%.7g,", points.end.y / gConvert.xyz);
		fprintf(pOut, "%.7g,", (points.end.z - pTL->TLLen) / gConvert.xyz);

		if(pWire->WDiaGa == (uint16_t)-1) {
			// There is only one scale factor in the GS card, so
			// we cannot show wire diameter in different units
			// than wire endpoints.
			//
			// NEC wants the radius, hence the divide-by-2.
			fprintf(pOut, "%.7g\n", (pWire->WireDia / 2.0) / gConvert.xyz);
		} else {
			// Calculate the wire diameter in meters, based on AWG
			// algorithm.  Works for wire GA 0 and thinner.
			//
			// Print it based on the user's scalefactor.
			//
			// NEC wants the radius, hence the divide-by-2.
			tmp = 0.005 * pow(92.0, ((36.0 - pWire->WDiaGa) / 39.0)) * 0.0254;
			fprintf(pOut, "%.7g\n", (tmp / 2.0) / gConvert.xyz);
		}
	}
	
	if(gConvert.xyz != 1.0) {
		fprintf(pOut, "GS %d,%d,%.7g\n", 0, 0, gConvert.xyz);
	}

        if (gNecVersion == 2)
		fprintf(pOut, "GE %d\n", 0);
        else
		fprintf(pOut, "GE %d\n", (-1));
}

// Print EX cards.
int
printExcitation(FILE *pOut)
{
	int i;
	RecType2 *pSrc;		// Record containing the source information.
	RecType2 *pWire;	// Record containing the wire information.
	int type;		// Type of source
	int wireNo;		// Number of the wire the source connects to.
	int segNo;		// Segment to connect the source to.

	// For a physical wire, this is the percentage along that wire where
	// the source connects.  We map it to a segment number.
	//
	// But for a virtual wire, this is actually an index into a table of
	// virtual wire tags.
	float percent;

	for(i = 0; i < gPointers.pRec1->NSrc; i++) {
		pSrc = gPointers.ppRec2[i];
		wireNo = pSrc->SWNr;
		percent = pSrc->SWPct;

		// Find the record containing the desired wire.
		if((wireNo > 0) && (wireNo <= gPointers.pRec1->NW)) {
			pWire = gPointers.ppRec2[wireNo - 1];
		} else {
			fprintf(stderr, "Source %d references wire %d, which doesn't exist\n", i + 1, wireNo);
			return -1;
		}

		// If this is a virtual wire, we look up its "virtual wire number".
		// The percentage is really an index into a table, rather than a
		// 
		if(wireNo == gVSegWire) {
			// Virtual wire.  Find virtual segment number.
			//segNo = gVirtualSegs[virtualIndex(gVSegCount, percent)];

			// We don't implement virtual wires.  It is easier to
			// use the extra wires added for the benefit of older
			// EZNEC implementations.
			segNo = virtualIndex(pWire->WSegs, percent);
		} else {
			// Real wire.  Find segment along wire from the percentage.
			segNo = virtualIndex(pWire->WSegs, percent);
		}

		switch(pSrc->Stype) {
			case 'I':
				if (gNecVersion == 4)
					type = 6;
				else
					type = 4;
				break;
			case 'J': // FIXME - not sure how to handle this yet.
			case 'W': // FIXME - not sure how to handle this yet.
			default:
				type = 0;
				break;
		}

		if (gNecVersion == 5 && pWire->SWPct < 50)
			segNo = -segNo;

		fprintf(pOut, "EX %d,%d,%d,%d,", type, wireNo, segNo, 0);

                // SMP_M seems to be an RMS magnitude -- WHY???
		fprintf(pOut, "%.7g,%.7g\n",
				pSrc->SMP_M * (cos(pSrc->SMP_Pdeg * PI / 180.0)*sqrt(2.0)),
				pSrc->SMP_M * (sin(pSrc->SMP_Pdeg * PI / 180.0)*sqrt(2.0)));
	}
}

// Print PQ card

int
printCharges(FILE *pOut)
{
	fprintf(pOut, "PQ 0\n");
}

// Print LD cards.  Also, synthesize cards representing wire losses.
int
printLoads(FILE *pOut)
{
	int i;
	RecType2 *pLoad;
	RecType2 *pWire;
	int wireNo;
	int segNo;
	int type;

	for(i = 0; i < gPointers.pRec1->NL; i++) {
		pLoad = gPointers.ppRec2[i];
		wireNo = pLoad->LWNr;

		if((wireNo > 0) && (wireNo <= gPointers.pRec1->NW)) {
			pWire = gPointers.ppRec2[wireNo - 1];
		} else {
			fprintf(stderr, "Load %d references wire %d, which doesn't exist\n", i + 1, wireNo);
			return -1;
		}

		segNo = virtualIndex(pWire->WSegs, pLoad->LWPct);

                if (gNecVersion == 5 && pWire->SWPct < 50)
			segNo = -segNo;

		if(gPointers.pRec1->LType == 'Z') {
			fprintf(pOut, "LD %d,%d,%d,%d,", 4, wireNo, segNo, 0);
			fprintf(pOut, "%.7g,%.7g\n", pLoad->LZ_R, pLoad->LZ_I);
		} else if(gPointers.pRec1->LType == 'R') {
			if(pLoad->strRLCType == 'P' || pLoad->strRLCType == 'T') {
				// Parallel
				type = 1;
			} else {
				// Series or unspecified.  Force series to
				// cover the unspecified case.
				pLoad->LConn = 'S';
				type = 0;
			}

			fprintf(pOut, "LD %d,%d,%d,%d,", type, wireNo, segNo, 0);
			fprintf(pOut, "%.7g,%.7g,%.7g\n", pLoad->sngR, pLoad->sngL, pLoad->sngC);
		} else {
			// Laplace
		}
	}

#if 0
	// Synthesize loads based on wire conductivity.
	if(gPointers.pRec1->WRho != 0 || gPointers.pRec1->WMu != 1) {
		for(i = 0; i < gPointers.pRec1->NW; i++) {
			fprintf(pOut, "LD %d,%d,%d,%d,", 5, i + 1, 0, 0);
			fprintf(pOut, "%.7g,%.7g\n", 1.0 / gPointers.pRec1->WRho, gPointers.pRec1->WMu);
		}
	}
#else
	// Load all segments the same way.
	fprintf(pOut, "LD %d,%d,%d,%d,", 5, 0, 0, 0);
	fprintf(pOut, "%.7g,%.7g\n", 1.0 / gPointers.pRec1->WRho, gPointers.pRec1->WMu);
#endif

	return 0;
}

// Print GN cards.
int
printGrounds(FILE *pOut)
{
	int type;
	RecType2 *pGnd;

	switch(gPointers.pRec1->Gtype) {
		case 'F':
			// F = free space
			fprintf(pOut, "GN %5d\n", -1);
			break;

		case 'P':
			// P = perfect
			fprintf(pOut, "GN %5d\n", 1);
			break;

		case 'R':
			// R = real
			//
			// If real, then we could also look at GAnal where:
			//
			// H means Sommerfeld (high accuracy)
			// M means MININEC
			//
			// We cannot do MININEC, so there is no advantage to
			// looking at GAnal
			//
			// There are two Sommerfeld methods, and we always
			// pick the higher quality one if we have the NEC 4
			// engine available.
			type = 0;
			if(gNecVersion == 4)
				type = 3;
			else if (gNecVersion == 2)
                                type = 2;

			pGnd = gPointers.ppRec2[0];

			fprintf(pOut, "GN %d,%d,%d,%d,", type, gPointers.pRec1->NR, 0, 0);
			fprintf(pOut, "%.7g,%.7g,", pGnd->MEps, pGnd->MSigma);
                        if (gNecVersion == 5)
                                fprintf(pOut, "%.7g,%.7g\n", 1.0, 0.0);
                        else
                                fprintf(pOut, "\n");

			if(gPointers.pRec1->Gtype == 'R') {
				if(gPointers.pRec1->NM == 2) {
					// FIXME radials, second ground medium
				}
			}
			break;
	}

	return 0;
}

// Print FR cards.
int
printFrequencies(FILE *pOut)
{
	if(gpFSB) {
		if(gpFSB->FStep == 0) {
			// Step size is zero, so there is only a single frequency.
			// We cannot use gpFSB->FStart because it is likely zero
			// also.
			if(gDebug) fprintf(stderr, "TCFreq %.7g\n", gFrequency);
			fprintf(pOut, "FR %d,%d,%d,%d,", 0, 1, 0, 0);
			fprintf(pOut, "%.7g,%.7g\n", gFrequency, 0.0);
		} else {
			int steps = 1 + (int)((gpFSB->FStop - gpFSB->FStart) / gpFSB->FStep);

			fprintf(pOut, "FR %d,%d,%d,%d,", 0, steps, 0, 0);
			fprintf(pOut, "%.7g,%.7g\n", gpFSB->FStart, gpFSB->FStep);
		}
	}

	return 0;
}

// Convert frequency in MHz to wavelength in meters.
double
freqToWavelength(double freq)
{
	return 299.792458 / freq;
}

// Print TL cards.
int
printTransmissionLines(FILE *pOut)
{
	int i;
	RecType2 *pTL;		// Record containing the TL information.
	RecType2 *pWire1;	// Record containing the side-1 wire information.
	RecType2 *pWire2;	// Record containing the side-2 wire information.
	uint16_t wireNo1;	// Number of the wire side-1 connects to.
	uint16_t wireNo2;	// Number of the wire side-2 connects to.
	int segNo1;		// Segment on wire-1.
	int segNo2;		// Segment on wire-2.
	double length;		// Length of line
	double y1r;		// Real shunt admittance on side-1
	double y1i;		// Imaginary shunt admittance on side-1
	double y2r;		// Real shunt admittance on side-2
	double y2i;		// Imaginary shunt admittance on side-2

	if(gDebug) fprintf(stderr, "%d TL\n", gPointers.pRec1->NT);

	// Start off synthetic wires at a safe number.  This has to match
	// what we used in printWires() earlier.
	gSyntheticWire = WIRE_MAX + 1;
	for(i = 0; i < gPointers.pRec1->NT; i++) {
		pTL = gPointers.ppRec2[i];
		wireNo1 = pTL->TLWNr1;
		wireNo2 = pTL->TLWNr2;
		if(gDebug) fprintf(stderr, "TL %d uses wire %d and wire %d\n", i + 1, wireNo1, wireNo2);

		// Check for TLs with two virtual ends.
		if((wireNo1 == SHORTED_END || wireNo1 == OPEN_END) && (wireNo2 == SHORTED_END || wireNo2 == OPEN_END)) {
			// This is bizarre - both ends of the TL are virtual.
			// Ignore this TL.
			continue;
		}

		// Assign a synthetic wire if needed.
		if(wireNo1 == SHORTED_END || wireNo1 == OPEN_END) {
			if(wireNo1 == SHORTED_END) {
				// Create a short via the admittance parameter.
				y1r = 1E12;
				y1i = 0.0;
			} else {
				// Create an open via the admittance parameter.
				y1r = 0.0;
				y1i = 0.0;
			}

			if(gDebug) fprintf(stderr, "TL %d references wire-1 %d, ", i + 1, wireNo1);

			wireNo1 = gSyntheticWire++;
			segNo1 = 1;
			if(gDebug) fprintf(stderr, "so use synthetic wire %d instead\n", wireNo1);
		} else {
			// A nice normal wire.
			y1r = 0.0;
			y1i = 0.0;
			pWire1 = gPointers.ppRec2[wireNo1 - 1];
			segNo1 = virtualIndex(pWire1->WSegs, pTL->TLWPct1);
		}

		// Assign a synthetic wire if needed.
		if(wireNo2 == SHORTED_END || wireNo2 == OPEN_END) {
			if(wireNo2 == SHORTED_END) {
				// Create a short via the admittance parameter.
				y2r = 1E12;
				y2i = 0.0;
			} else {
				// Create an open via the admittance parameter.
				y2r = 0.0;
				y2i = 0.0;
			}

			if(gDebug) fprintf(stderr, "TL %d references wire-2 %d, ", i + 1, wireNo2);

			wireNo2 = gSyntheticWire++;
			segNo2 = 1;
			if(gDebug) fprintf(stderr, "so use synthetic wire %d instead\n", wireNo2);
		} else {
			// A nice normal wire.
			y2r = 0.0;
			y2i = 0.0;
			pWire2 = gPointers.ppRec2[wireNo2 - 1];
			segNo2 = virtualIndex(pWire2->WSegs, pTL->TLWPct2);
		}

		if(pTL->TLLen > 0) {
			// Positive TLLen is length in meters.
			length = pTL->TLLen;
			if(gDebug) fprintf(stderr, "TL Length %.7g, ", pTL->TLLen / gConvert.xyz);
		} else if(pTL->TLLen < 0) {
			// Negative TLLen implies length in degrees.
			double wl = freqToWavelength(gFrequency);
			length = pTL->TLVF * wl * -pTL->TLLen / 360.0;
			if(gDebug) fprintf(stderr, "TL Length %.7g degrees, meters %.7g, ", -pTL->TLLen, length);
		} else {
			// 0 TLLen means "use physical distance between
			// endpoints".
			length = 0.0;
			if(gDebug) fprintf(stderr, "TL Length actual distance, ");
		}

		if(gDebug) fprintf(stderr, "TL VF %.7g\n", pTL->TLVF);

		// A negative TLZ0 means to reverse the connections at one
		// end, thus introducing a 180 degree phase shift.
		fprintf(pOut, "TL %d,%d,%d,%d,", wireNo1, segNo1, wireNo2, segNo2);
		fprintf(pOut, "%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n",
				pTL->TLZ0, length, y1r, y1i, y2r, y2i);
	}

	return 0;
}

// Print NT cards.
int
printLNetworks(FILE *pOut)
{
	int i;
	RecType2 *pWire1;	// Record containing the side-1 wire information.
	RecType2 *pWire2;	// Record containing the side-2 wire information.
	uint16_t wireNo1;	// Number of the wire side-1 connects to.
	uint16_t wireNo2;	// Number of the wire side-2 connects to.
	int segNo1;		// Segment on wire-1.
	int segNo2;		// Segment on wire-2.

	LNetBlockA	*pA = gLNB.pA;
	LNetBlockB	*pB = gLNB.pB;
	LNetBlockC	*pC = gLNB.pC;
	LNetBlockD	*pD = gLNB.pD;
	LNetBlockE	*pE = gLNB.pE;
	LNetBlockF	*pF = gLNB.pF;
	LNetBlockG	*pG = gLNB.pG;

	// For a physical wire, this is the percentage along that wire where
	// the TL connects.  We map it to a segment number.
	//
	// But for a virtual wire, this is actually an index into a table of
	// virtual wire tags.
	float percent1;
	float percent2;

	if(gPointers.pRec1->NLNet && pA) {
		fprintf(stderr, "%d LNets\n", pA->NL);
		for(i = 1; i <= pA->NL; i++) {
			wireNo1 = pB[i].P1WNr;
			wireNo2 = pB[i].P2WNr;
			fprintf(stderr, "LNet %d uses wire-1 %d and wire-2 %d\n", i, wireNo1, wireNo2);

			if((wireNo1 > 0) && (wireNo1 <= gPointers.pRec1->NW)) {
				pWire1 = gPointers.ppRec2[wireNo1 - 1];
			} else {
				fprintf(stderr, "TL %d references wire-1 %d, which doesn't exist\n", i + 1, wireNo1);
				return -1;
			}

			if((wireNo2 > 0) && (wireNo2 <= gPointers.pRec1->NW)) {
				pWire2 = gPointers.ppRec2[wireNo2 - 1];
			} else {
				fprintf(stderr, "TL %d references wire-2 %d, which doesn't exist\n", i + 1, wireNo2);
				return -1;
			}

			percent1 = pC[i].P1WPct;
			percent2 = pC[i].P2WPct;

			// If wire-1 is a virtual wire, we look up its "virtual wire number".
			// The percentage is really an index into a table, rather than a
			// 
			if(wireNo1 == gVSegWire) {
				fprintf(stderr, "LNet %d wire-1 %d matches vseg %d\n", i, wireNo1, gVSegWire);
				// Virtual wire.  Find virtual segment number.
				segNo1 = gVirtualSegs[virtualIndex(gVSegCount, percent1)];

				segNo1 = virtualIndex(pWire1[i].WSegs, percent1);
			} else {
				fprintf(stderr, "LNet %d wire-1 %d doesn't match vseg %d\n", i, wireNo1, gVSegWire);
				// Real wire.  Find segment along wire from the percentage.
				segNo1 = virtualIndex(pWire1[i].WSegs, percent1);
			}

			// If wire-2 is a virtual wire, we look up its "virtual wire number".
			// The percentage is really an index into a table, rather than a
			// 
			if(wireNo2 == gVSegWire) {
				fprintf(stderr, "LNet %d wire-2 %d matches vseg %d\n", i, wireNo2, gVSegWire);
				// Virtual wire.  Find virtual segment number.
				segNo2 = gVirtualSegs[virtualIndex(gVSegCount, percent2)];

				segNo2 = virtualIndex(pWire2[i].WSegs, percent2);
			} else {
				fprintf(stderr, "LNet %d wire-2 %d doesn't match vseg %d\n", i, wireNo2, gVSegWire);
				// Real wire.  Find segment along wire from the percentage.
				segNo2 = virtualIndex(pWire2[i].WSegs, percent2);
			}

			fprintf(pOut, "NT %d,%d,%d,%d,", wireNo1, segNo1, wireNo2, segNo2);

			if(gPointers.pRec1->LNetType == 'Z') {
				double complex z1 = gLNB.pD[i].B1R + gLNB.pD[i].B1X * I;
				double complex z2 = gLNB.pD[i].B2R + gLNB.pD[i].B2X * I;
				double complex y1 = 1.0 / z1;
				double complex y2 = 1.0 / z2;

				// B1 is in series
				// B2 is in parallel
				double complex y12 = -y1;
				double complex y11 = y1;
				double complex y22 = y2 - y1;
				if(gDebug) fprintf(stderr, "B1R %.7g, ", gLNB.pD[i].B1R);
				if(gDebug) fprintf(stderr, "B1X %.7g, ", gLNB.pD[i].B1X);
				if(gDebug) fprintf(stderr, "B2R %.7g, ", gLNB.pD[i].B2R);
				if(gDebug) fprintf(stderr, "B2X %.7g\n", gLNB.pD[i].B2X);

				fprintf(pOut, "%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n",
						creal(y11), cimag(y11),
						creal(y12), cimag(y12),
						creal(y22), cimag(y22));
			} else {
				// B1 is in series
				// B2 is in parallel
				//
				// This is a problem, because it has to be frequency-specific.
				// So we really cannot produce a set of cards to run over a
				// range of frequencies.  Instead, we'd have to create a
				// separate card deck for each frequency, I think,
				if(gDebug) fprintf(stderr, "B1rlcR %.7g, ", gLNB.pF[i].B1rlcR);
				if(gDebug) fprintf(stderr, "B1rlcL %.7g, ", gLNB.pF[i].B1rlcL * 1000000);
				if(gDebug) fprintf(stderr, "B1rlcC %.7g, ", gLNB.pF[i].B1rlcC * 1000000000000);
				if(gDebug) fprintf(stderr, "B1rlcF %.7g, ", gLNB.pF[i].B1rlcF);
				if(gDebug) fprintf(stderr, "B2rlcR %.7g, ", gLNB.pF[i].B2rlcR);
				if(gDebug) fprintf(stderr, "B2rlcL %.7g, ", gLNB.pF[i].B2rlcL * 1000000);
				if(gDebug) fprintf(stderr, "B2rlcC %.7g, ", gLNB.pF[i].B2rlcC * 1000000000000);
				if(gDebug) fprintf(stderr, "B2rlcF %.7g\n", gLNB.pF[i].B2rlcF);

				static char *Config[] = { "Ser", "Par", "Trap"};
				char B1RLCType_Value[STR_LEN];
				char B2RLCType_Value[STR_LEN];
				switch(gLNB.pG[i].B1RLCType) {
					case 'S': strlcpy(B1RLCType_Value, Config[0], STR_LEN); break;
					case 'P': strlcpy(B1RLCType_Value, Config[1], STR_LEN); break;
					case 'T': strlcpy(B1RLCType_Value, Config[2], STR_LEN); break;
					default:  strlcpy(B1RLCType_Value, "?", STR_LEN); break;
				}
				switch(gLNB.pG[i].B2RLCType) {
					case 'S': strlcpy(B2RLCType_Value, Config[0], STR_LEN); break;
					case 'P': strlcpy(B2RLCType_Value, Config[1], STR_LEN); break;
					case 'T': strlcpy(B2RLCType_Value, Config[2], STR_LEN); break;
					default:  strlcpy(B2RLCType_Value, "?", STR_LEN); break;
				}
				if(gDebug) fprintf(stderr, "ConfigB1 %s, ConfigB2 %s\n", B1RLCType_Value, B2RLCType_Value);

				fprintf(pOut, "%.7g,%.7g,%.7g,%.7g,%.7g,%.7g\n",
						0.0, 0.0,	// Y11R and Y11I
						0.0, 0.0,	// Y12R and Y12I
						0.0, 0.0);	// Y22R and Y22I
			}
		}
	}

	return 0;
}

// Print RP card.
int
printReport(FILE *pOut)
{
	double step;

	if (gPointers.pRec1->PType == '3' || gPointers.pRec1->PType == '2')
                step = gPointers.pRec1->PStep3D;
        else
                step = gPointers.pRec1->PStep;

	// The + 1 on theta is to make xnecview happy.
	// The - 1 on phi is to make xnecview happy.
	fprintf(pOut, "RP %d,%d,%d,%d,", 0, ((int)round(90.0 / step) + 1), ((int)round(360.0 / step) - 1), 1001);
	fprintf(pOut, "%.7g,%.7g,%.7g,%.7g,%.7g\n", 0.0, 0.0, step, step, 0.0);
	
}

// Process an EZNEC file.
void
process(
		char *InputFile,
		char *OutputFile
		)
{
	int		i;
	FILE		*pOut;

	// Create an output file, if requested.  Otherwise we will just use stdout.
	if(OutputFile) {
		errno = 0;
		if((pOut = fopen(OutputFile, "w")) == NULL) {
			fprintf(stderr, "Cannot open/create %s for output, errno %d\n", OutputFile, errno);
			exit(1);
		}
	} else {
		pOut = stdout;
	}

	// Map the input file.
	if(mmapInput(InputFile) < 0) {
		// Error messages already printed via mmapInput.
		exit(1);
	}

	// Map the globals.
	if(mapRecType1() < 0) {
		// Error messages already printed via mmapInput.
		exit(1);
	}
	
	// Map all the RecType2 blocks.
	if(mapRecType2() < 0) {
		// Error messages already printed via mmapInput.
		exit(1);
	}

	// Map the RecType3 block.
	if(mapRecType3() < 0) {
		// Error messages already printed via mmapInput.
		exit(1);
	}

	// Map all variable-length blocks.
	if(mapVarBlocks() < 0) {
		// Error messages already printed via mmapInput.
		exit(1);
	}

	// We need to know about virtual wires before printing.
	loadVirtualWires();

	printCMCE(pOut);
	printWires(pOut);
	printLoads(pOut);
	printExcitation(pOut);
        printCharges(pOut);
	printTransmissionLines(pOut);
	printLNetworks(pOut);
	printGrounds(pOut);
	printFrequencies(pOut);
	printReport(pOut);

	if(gpWIB) {
		int i;

		if(gDebug) fprintf(stderr, "\n");
		if(gDebug) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(gDebug) fprintf(stderr, "@@ Have Wire Insulation Block                                               @@\n");
		if(gDebug) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(gDebug) fprintf(stderr, "\n");

		for(i = 0; i < gpWIB->NumWires; i++) {
			if(gDebug) fprintf(stderr, "Dielectric C %.7g, ", gpWIB->Wires[i].DielC);
			if(gDebug) fprintf(stderr, "Thickness %.7g\n", gpWIB->Wires[i].Thk / gConvert.wdiam);
			if(gDebug) fprintf(stderr, "Loss Tangent %.7g\n", gpWIB->Wires[i].LTan);
		}
	}

	if(gpTLLB) {
		int i;
		float *pFloat;

		if(gDebug) fprintf(stderr, "\n");
		if(gDebug) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(gDebug) fprintf(stderr, "@@ Have Transmission Line Loss Parameters                                   @@\n");
		if(gDebug) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(gDebug) fprintf(stderr, "\n");

		pFloat = &gpTLLB->Loss[0];
		for(i = 1; i <= gPointers.pRec1->NT; i++) {
			if(gDebug) fprintf(stderr, "Loss %.7g\n", *pFloat++ * gConvert.tldB * 100);
		}

		for(i = 1; i <= gPointers.pRec1->NT; i++) {
			if(gDebug) fprintf(stderr, "LossFreq %.7g\n", *pFloat++);
		}

		i = gPointers.pRec1->NT;
	}

	if(gTB.pA) {
		int i;

		if(gDebug) fprintf(stderr, "\n");
		if(gDebug) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(gDebug) fprintf(stderr, "@@ Have Transformer Parameters                                              @@\n");
		if(gDebug) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(gDebug) fprintf(stderr, "\n");

		for(i = 0; i <= gTB.pA->NX; i++) {
			if(gDebug) fprintf(stderr, "P1WNr %d, ", gTB.pB[i].P1WNr);
			if(gDebug) fprintf(stderr, "P2WNr %d\n", gTB.pB[i].P2WNr);
		}

		for(i = 0; i <= gTB.pA->NX; i++) {
			if(gDebug) fprintf(stderr, "P1WPct %.7g, ", gTB.pC[i].P1WPct);
			if(gDebug) fprintf(stderr, "P2WPct %.7g\n", gTB.pC[i].P2WPct);
		}

		for(i = 0; i <= gTB.pA->NX; i++) {
			if(gDebug) fprintf(stderr, "P1RelZ %.7g, ", gTB.pD[i].P1RelZ);
			if(gTB.pD[i].P2RelZ >= 0) {
				if(gDebug) fprintf(stderr, "P2RelZ %.7g N\n", gTB.pD[i].P2RelZ);
			} else {
				if(gDebug) fprintf(stderr, "P2RelZ %.7g R\n", -gTB.pD[i].P2RelZ);
			}
		}
	}

}

void
usage(char *prog)
{
	fprintf(stderr, "%s -d -i input_file -o output_file -n nec_version\n", prog);
	fprintf(stderr, "\n");
	fprintf(stderr, "You can provide the input_file with or without the -i\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "If you don't specify an output_file, then output will\n");
	fprintf(stderr, "go to the terminal\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "nec_version can be 2 or 4.  This influences various\n");
	fprintf(stderr, "card generation parameters.\n");
}

int
main(
		int argc,
		char *argv[]
		)
{
	int opt;
	char *pInput = 0;
	char *pOutput = 0;

	gNecVersion = 5; // default

	while((opt = getopt(argc, argv, "di:o:n:")) != -1) {
		switch(opt) {
			case 'd':
				gDebug = 1;
				break;

			case 'i':
				pInput = optarg;
				break;

			case 'o':
				pOutput = optarg;
				break;

			case 'n':
				gNecVersion = atoi(optarg);
				if(gNecVersion != 2 && gNecVersion != 4 && gNecVersion != 5) {
					fprintf(stderr, "Invalid NEC version\n\n");
					usage(argv[0]);
					exit(1);
				}
				break;

			default: /* '?' */
				usage(argv[0]);
				exit(1);
		}
	}

	if (optind < argc) {
		// Override any -i.
		pInput = argv[optind];
	}

	process(pInput, pOutput);

	exit(0);
}
