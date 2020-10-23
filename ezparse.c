#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <string.h>
#include <unistd.h>

#define STR_LEN		16
#define TITLE_LEN	30

#define PRIMARY_BS	170 // size of primary blocks

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
		if(Debug_Flag) fprintf(stderr, "map " #type " at 0x%x through 0x%x\n",	\
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
		if(Debug_Flag) fprintf(stderr, "map " #type " at 0x%x through 0x%x\n",	\
				offset_in_bytes, offset_in_bytes + length - 1);		\
		if((offset_in_bytes + length) > gInputFileSize) {			\
			fprintf(stderr, "Cannot map - runs off end of file\n");		\
			exit(1);							\
		}									\
		(lval) = OFFSET_TO(type, (void_pointer), (offset_in_bytes));		\
	} while(0)

#ifndef true
#define true 1
#endif

#ifndef false
#define false 0
#endif

// Globals
char SrcType_Value[STR_LEN];
char LoadConfig_Value[STR_LEN];
char LoadType_Value[STR_LEN];
char B1RLCType_Value[STR_LEN];
char B2RLCType_Value[STR_LEN];
int Debug_Flag = 0;

// NB: We only declare structs to be packed if they really need it.
// Otherwise, the compiler would give us "unaligned pointer" warnings
// in some cases.
//
// Also, we sometimes split blocks to separate items of the same type.

// RecType1 - globals
//
// 170 bytes long
typedef struct __attribute__((__packed__)) {
	uint16_t OldMaxMWSL;	// Old Maximum of NW, NSrc, NL, NT, NM
	uint16_t NM;		// GN Number of media, usually 1, occasionally 2
	uint8_t BdryType;	// GN Set to 'R' if media2 is radials, else 'X'
	float FMHz;		// FR FMHZ, starting frequency or single frequency
	uint8_t OldPType;	// (A)zimuth or (E)levation; write ‘E’ if plot type is 3D
	float Pangle;		// RP Plot angle - could be azimuth or elevation
	float PStep;		// RP Plot angle steps
	char Title[TITLE_LEN];	// CM Title
	uint16_t OldNW;		// old GW Number of wires
	uint16_t NSrc;		// EX Number of sources
	uint16_t NL;		// LD Number of loads
	uint8_t Gtype;		// GN Ground type 'P' perfect, 'R' real, 'F' free space
	uint16_t NR;		// Number of radials
	float RDia;		// Radial diameter
	uint16_t RDiaGa;	// Radial wire gauge - negative means "ignore"
	uint16_t Ck;		// Check Constant = 1304 always
	uint8_t Units;		// M meters, L millimeters, F feet, I inches, W wavelength
	uint8_t PRange;		// 2D Plot range: (F)ull or (P)artial
	uint8_t OldPPol;	// Polarizations. shown in 2D plot: (T)ot only, or (A)ll [VHT]
	float ORdb;		// dBi of outer ring of 2D plot (1304 = auto scaled)
	float PStart;		// Start angle for 2D plot in degrees
	float PEnd;		// End angle for 2D plot in degrees
	uint8_t OldLType;	// Load type: S (Laplace) or Z. Write ‘Z’ if load type is RLC
	float GWDist;		// Ground wave distance (used by EZNEC Pro only)
	float GWZ;		// Ground wave elevation (z coord) (used by EZNEC Pro only)
	uint8_t LNetType;	// L Network type: (Z) - RX; (R) - RLC
	float AnalRes;		// Write the same value into this field as into PStep
	float RefdB;		// Field strength reference level in dBi
	float WRho;		// Wire resistivity in ohm-m
	float WMu;		// Wire relative permeability
	uint16_t NP;		// 0 - used by earlier programs
	uint16_t ArrayFiles;	// 0 - used by earlier programs
	float SWRZ0;		// User-defined Z0 for SWR outputs
	uint16_t VerCode;	// Writing program version code: Write decimal 51 into this field
	uint8_t DiaUnits;	// Diameter units, usually M or L millimeters, F or I inches, W wavelength
	uint8_t LType;		// Load type: (Z) - RX; (R) - RLC; (L) - Laplace
	uint16_t PPol;		// 2D plot polarizations shown - 1-Tot only; 2-V,H,T; 3-V,H; 4-R & L circ, 5-R circ, 6-L circ, 7-Lin Maj/Min
	uint16_t NT;		// TL Number of transmission lines
	uint8_t PType;		// Plot type: (A)z, (E)l, (2)D or (3)D
	uint8_t GAnal;		// Real gnd analysis type: (H)igh accuracy, (M)ININEC
	uint32_t MaxMWSL;	// Maximum of Nw, NSrc, NL, NT, NM
	uint32_t NW;		// GW Number of wires
	float dBSum;		// Always write 0
	float PStep3D;		// 3D plot step size
	uint32_t MiscFlags;	// Bit 0 =  Allow ctr of seg within another wire, Bit 1 =  allow plane wave excitation with no conventional sources
	uint32_t PgmVerCode;	// 50000200 old, or 62000000 currently
	float LinRange2D;	// Range of 2D linear plot scale
	uint16_t NX;		// Number of transformers
	uint16_t NN;		// Number of Y parameter networks
	uint16_t NLNet;		// Number of LNets
	uint16_t NVirt;		// Number of Virtual segments
	uint16_t Reserved1;	// Write 0
	uint8_t	HAGndType;	// “High” accuracy gnd type (H)igh or e(X)tended
	char Reserved2[7];	// Write 0
} RecType1;

// RecType2 - per-wire/source/load etc.
//
// Also has some overflow from RecType1, so some things get repeated that
// don't need to be.
//
// 170 bytes long
typedef struct __attribute__((__packed__)) {
	// Medium (ground) parameters.
	float MSigma;		// GN SIG (conductivity in Siemens, 0.005 is typical)
	float MEps;		// GN EPSR (dielectric constant, 13 is typical)
	float MCoord;		// GN Medium R or X coordinate of medium
	float MHt;		// GN Medium height

	// Wire parameters.
	float WEnd1_X;		// GW end 1 X
	float WEnd1_Y;		// GW end 1 Y
	float WEnd1_Z;		// GW end 1 Z
	float WEnd2_X;		// GW end 2 X
	float WEnd2_Y;		// GW end 2 Y
	float WEnd2_Z;		// GW end 2 Z
	float WireDia;		// GW wire diameter
	uint16_t WDiaGa;	// GW wire gauge (negative if not specified as gauge)
	uint16_t WSegs;		// GW wire segments

	// Source parameters.
	uint16_t SWNr;		// EX source wire number
	float SWPct;		// EX source wire percentage from end 1 (position on wire)
	float SMP_M;		// EX source magnitude, volts or amperes
	float SMP_Pdeg;		// EX source phase angle in degrees
	uint8_t Stype;		// EX source type: V voltage, I current, W split voltage, J split current

	// Load parameters.
	uint16_t LWNr;		// LD load wire number
	float LWPct;		// LD load wire percentage from end 1(position on wire)
	float LZ_R;		// LD load resistance (real part)
	float LZ_I;		// LD load reactance (imaginary part)
	float LNum_S0;		// 0-order numerator coeff. of Laplace load impedance
	float LNum_S1;		// 1-order numerator coeff. of Laplace load impedance
	float LNum_S2;		// 2-order numerator coeff. of Laplace load impedance
	float LNum_S3;		// 3-order numerator coeff. of Laplace load impedance
	float LNum_S4;		// 4-order numerator coeff. of Laplace load impedance
	float LNum_S5;		// 5-order numerator coeff. of Laplace load impedance
	float LDen_S0;		// 0-order denominator coeff. of Laplace load impedance
	float LDen_S1;		// 1-order denominator coeff. of Laplace load impedance
	float LDen_S2;		// 2-order denominator coeff. of Laplace load impedance
	float LDen_S3;		// 3-order denominator coeff. of Laplace load impedance
	float LDen_S4;		// 4-order denominator coeff. of Laplace load impedance
	float LDen_S5;		// 5-order denominator coeff. of Laplace load impedance

	// Transmission line parameters.
	uint16_t TLWNr1;	// TL wire #1 endpoint, or -1 (shorted stub) or -2 (open stub)
	float TLWPct1;		// TL wire #1 percentage from end 1 (position on wire)
	uint16_t TLWNr2;	// TL wire #2 endpoint, or -1 (shorted stub) or -2 (open stub)
	float TLWPct2;		// TL wire #2 percentage from end 1 (position on wire)
	float TLZ0;		// TL impedance, Z0, negated if reversed connection
	float TLLen;		// TL length, 0 => physical length; neg => degrees; pos => meters
	float TLVF;		// TL velocity factor

	// Load parameters (additional).
	uint8_t LConn;		// LD connection type: S = series, P = parallel

	// Load parameters (only for RLC loads - LType == 'R').
	uint8_t strRLCType;	// LD S = series, P = parallel, T = trap
	float sngR;		// LD Resistance in Ohms (0 means "not present")
	float sngL;		// LD Inductance in Henries (0 means "not present")
	float sngC;		// LD Capacitance in Farads (0 means "not present")
	float sngRFreqMHz;	// LD Frequency
	char Reserved[3];	// Write zeros
} RecType2;

// RecType3 - Near Field parameters
//
// 170 bytes long
typedef struct __attribute__((__packed__)) {
	uint8_t NFType;		// E or H, electrical field vs magnetic field
	uint8_t NFCoordType;	// S (spherical) or C (cartesian)
	float NF1Start;		// Starting X or rho
	float NF1Stop;		// Stopping X or rho
	float NF1Step;		// Step size X or rho
	float NF2Start;		// Starting Y or theta (azimuth in degrees)
	float NF2Stop;		// Stopping Y or theta (azimuth in degrees)
	float NF2Step;		// Step size Y or theta
	float NF3Start;		// Starting Z or phi (zenith in degrees)
	float NF3Stop;		// Stopping Z or phi (zenith in degrees)
	float NF3Step;		// Step size Z or phi
	char Reserved[132];	// Write zeros
} RecType3;

// RecType4 - reserved
//
// 170 bytes long
typedef struct __attribute__((__packed__)) {
	char Reserved[170];	// Not used for anything currently
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
	uint16_t BlockType;	// Block type 11, 12, 14, 15, 17, 18, 31, 101
	uint32_t BlockLen;	// Length, includes this header
	uint8_t BlockRev;	// Block format revision
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
	uint32_t DataLen;	// Set to 16
	uint16_t FIFLen;	// Length of Frequency Input file path and name
	uint16_t PPFLen;	// Length of Pattern Plot file path and name
	uint16_t SCPFLen;	// Length of Smith Chart Program path and name
	uint16_t DFLen;		// Length of Data file path and name
	uint16_t AFLen;		// Reserved
	uint32_t Flags;		// Bits are defined below
	float FStart;		// Starting frequency
	float FStop;		// Ending frequency
	float FStep;		// Frequency step size
	char FreqInFile[1];	// Frequency Input file path and name
	char PatPlotFile[1];	// Pattern Plot file path and name
	char SCPFile[1];	// PSmith Chart Program path and name
	char DataFile[1];	// PData file path and name
	uint16_t EndTest;	// Always set to 12345
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
	uint32_t NumWires;	// Number of wires in the following array
	struct {
		float DielC;	// Wire Insulation Dielectric (permittivity)
		float Thk;	// Wire Insulation Thickness (meters)
		float LTan;	// Loss Tangent
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
	uint32_t NumLines;	// Number of transmission lines in the arrays
	float Loss[1];		// Transmission line loss (dB / 100 feet)
	float LossFreq[1];	// Transmission line loss frequency
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
	uint32_t NX;		// Number of transformers
} TransformerBlockA;

typedef struct {
	uint32_t P1WNr;		// Port 1 wire number
	uint32_t P2WNr;		// Port 2 wire number
} TransformerBlockB;

typedef struct {
	float P1WPct;		// Port 1 percentage (position on wire)
	float P2WPct;		// Port 2 percentage (position on wire)
} TransformerBlockC;

typedef struct {
	float P1RelZ;		// Port 1 impedance (if negative, reverse polarity)
	float P2RelZ;		// Port 2 impedance (if negative, reverse polarity
} TransformerBlockD;

// L Network Parameters
//
// This block consists of five variable-length arrays.  Additionally, the
// arrays are really two-dimensional, because an L-network has two ports.
// There is no way to create a C structure like that, so we instead split
// the block into seven pieces (A-G).
typedef struct {
	uint32_t NL;		// Number of L-networks
} LNetBlockA;

typedef struct {
	uint32_t P1WNr;		// Port 1 wire number
	uint32_t P2WNr;		// Port 2 wire number
} LNetBlockB;

typedef struct {
	float P1WPct;		// Port 1 percentage (position on wire)
	float P2WPct;		// Port 2 percentage (position on wire)
} LNetBlockC;

// Using R +/- jX
typedef struct {
	float B1R;		// Port 1 resistance
	float B1X;		// Port 1 reactance
	float B2R;		// Port 2 resistance
	float B2X;		// Port 2 reactance
} LNetBlockD;

typedef struct {
	uint32_t NrRLC;		// WEzT1.NLNet if using RLC, else 0
} LNetBlockE;

// Using RLC at a frequency
typedef struct {
	float B1rlcR;		// Port 1 resistance
	float B2rlcR;		// Port 2 resistance
	float B1rlcL;		// Port 1 inductance (Henries)
	float B2rlcL;		// Port 2 inductance (Henries)
	float B1rlcC;		// Port 1 capacitance (Farads)
	float B2rlcC;		// Port 2 capacitance (Farads)
	float B1rlcF;		// Port 1 frequency
	float B2rlcF;		// Port 2 frequency
} LNetBlockF;

typedef struct {
	uint8_t B1RLCType;	// Port 1 hookup: S = series, P = parallel, T = trap
	uint8_t B2RLCType;	// Port 2 hookup: S = series, P = parallel, T = trap
} LNetBlockG;

// Virtual Segments - used to avoid modeling wires just to insert a source
// or load on a transmission line.  Perhaps other uses as well.  EZNEC
// converts these to real wires far away for compatibility with older
// versions of the program.
//
// We don't really know at compile time how big the VSegNr array might
// be.  Fortunately, it is the last (and only) variable length field.
typedef struct {
	uint32_t VWnr;		// Wire number to be replaced with the segment number
	uint32_t VSegNr[1];	// Segment number
} VirtSegmentBlock;

typedef struct {
	TransformerBlockA		*pA;
	TransformerBlockB		*pB;
	TransformerBlockC		*pC;
	TransformerBlockD		*pD;
} TransformerBlock;

typedef struct {
	LNetBlockA			*pA;
	LNetBlockB			*pB;
	LNetBlockC			*pC;
	LNetBlockD			*pD;
	LNetBlockE			*pE;
	LNetBlockF			*pF;
	LNetBlockG			*pG;
} LNetBlock;

typedef struct {
	BlkHeader			*pH;
	union {
		FreqSweepBlk		*pFSB;
		WireInsBlock		*pWIB;
		TLineLossBlock		*pTLLB;
		TransformerBlock	*pTB;
		LNetBlock		*pLNB;
		VirtSegmentBlock	*pVSB;
	} u;
} BLOCK;

typedef struct {
	RecType1			*pRec1;
	RecType2			**ppRec2;
	RecType3			*pRec3;
	RecType4			*pRec4;
	BLOCK				**ppBlks;
} POINTERS;

typedef struct {
	double				xyz;		// For converting endpoints
	double				wdiam;		// For converting wire diameters
	double				tldB;		// For converting transmission line dB
} CONVERSION_FACTORS;

double			PI = 4.0 * atan2(1.0, 1.0);

void			*gIMap;				// Pointer to mmapped input file.
off_t			gInputFileSize;			// Size of input file in bytes.
int			gStartVarLenBlocks;		// Offset to the first varlen block.
POINTERS		gPointers;			// Pointers to all records and blocks.
CONVERSION_FACTORS	gConvert;			// Conversion factors
double			gFrequency;			// Frequency in MHz

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

void
dumpRecType1(RecType1 *p)
{
	if(!Debug_Flag) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump RecType1                                                            @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "RecType1: OldMaxMWSL = %d\n", p->OldMaxMWSL);
	fprintf(stderr, "RecType1: NM = %d\n", p->NM);
	fprintf(stderr, "RecType1: BdryType = %c\n", p->BdryType);
	fprintf(stderr, "RecType1: FMHz = %f\n", (double)p->FMHz);
	fprintf(stderr, "RecType1: OldPType = %d\n", p->OldPType);
	fprintf(stderr, "RecType1: Pangle = %f\n", (double)p->Pangle);
	fprintf(stderr, "RecType1: PStep = %f\n", (double)p->PStep);
	fprintf(stderr, "RecType1: Title = "); printStr(p->Title, TITLE_LEN);
	fprintf(stderr, "RecType1: OldNW = %d\n", p->OldNW);
	fprintf(stderr, "RecType1: NSrc = %d\n", p->NSrc);
	fprintf(stderr, "RecType1: NL = %d\n", p->NL);
	fprintf(stderr, "RecType1: Gtype = %d\n", p->Gtype);
	fprintf(stderr, "RecType1: NR = %d\n", p->NR);
	fprintf(stderr, "RecType1: RDia = %f\n", (double)p->RDia);
	fprintf(stderr, "RecType1: RDiaGa = %d\n", p->RDiaGa);
	fprintf(stderr, "RecType1: Ck = %d\n", p->Ck);
	fprintf(stderr, "RecType1: Units = %d\n", p->Units);
	fprintf(stderr, "RecType1: PRange = %d\n", p->PRange);
	fprintf(stderr, "RecType1: OldPPol = %d\n", p->OldPPol);
	fprintf(stderr, "RecType1: ORdb = %f\n", (double)p->ORdb);
	fprintf(stderr, "RecType1: PStart = %f\n", (double)p->PStart);
	fprintf(stderr, "RecType1: PEnd = %f\n", (double)p->PEnd);
	fprintf(stderr, "RecType1: OldLType = %d\n", p->OldLType);
	fprintf(stderr, "RecType1: GWDist = %f\n", (double)p->GWDist);
	fprintf(stderr, "RecType1: GWZ = %f\n", (double)p->GWZ);
	fprintf(stderr, "RecType1: LNetType = %d\n", p->LNetType);
	fprintf(stderr, "RecType1: AnalRes = %f\n", (double)p->AnalRes);
	fprintf(stderr, "RecType1: RefdB = %f\n", (double)p->RefdB);
	fprintf(stderr, "RecType1: WRho = %f\n", (double)p->WRho);
	fprintf(stderr, "RecType1: WMu = %f\n", (double)p->WMu);
	fprintf(stderr, "RecType1: NP = %d\n", p->NP);
	fprintf(stderr, "RecType1: ArrayFiles = %d\n", p->ArrayFiles);
	fprintf(stderr, "RecType1: SWRZ0 = %f\n", (double)p->SWRZ0);
	fprintf(stderr, "RecType1: VerCode = %d\n", p->VerCode);
	fprintf(stderr, "RecType1: DiaUnits = %d\n", p->DiaUnits);
	fprintf(stderr, "RecType1: LType = %d\n", p->LType);
	fprintf(stderr, "RecType1: PPol = %d\n", p->PPol);
	fprintf(stderr, "RecType1: NT = %d\n", p->NT);
	fprintf(stderr, "RecType1: PType = %d\n", p->PType);
	fprintf(stderr, "RecType1: GAnal = %d\n", p->GAnal);
	fprintf(stderr, "RecType1: MaxMWSL = %d\n", p->MaxMWSL);
	fprintf(stderr, "RecType1: NW = %d\n", p->NW);
	fprintf(stderr, "RecType1: dBSum = %f\n", (double)p->dBSum);
	fprintf(stderr, "RecType1: PStep3D = %f\n", (double)p->PStep3D);
	fprintf(stderr, "RecType1: MiscFlags = %d\n", p->MiscFlags);
	fprintf(stderr, "RecType1: PgmVerCode = %d\n", p->PgmVerCode);
	fprintf(stderr, "RecType1: LinRange2D = %f\n", (double)p->LinRange2D);
	fprintf(stderr, "RecType1: NX = %d\n", p->NX);
	fprintf(stderr, "RecType1: NN = %d\n", p->NN);
	fprintf(stderr, "RecType1: NLNet = %d\n", p->NLNet);
	fprintf(stderr, "RecType1: NVirt = %d\n", p->NVirt);
	fprintf(stderr, "RecType1: Reserved1 = %d\n", p->Reserved1);
	fprintf(stderr, "RecType1: HAGndType = %d\n", p->HAGndType);
	//fprintf(stderr, "RecType1: Reserved[7] = %d\n", p->Reserved[7]);

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpRecType2(RecType2 *p)
{
	if(!Debug_Flag) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump RecType2                                                            @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "RecType2: MSigma = %f\n", (double)p->MSigma);
	fprintf(stderr, "RecType2: MEps = %f\n", (double)p->MEps);
	fprintf(stderr, "RecType2: MCoord = %f\n", (double)p->MCoord);
	fprintf(stderr, "RecType2: MHt = %f\n", (double)p->MHt);
	fprintf(stderr, "RecType2: WEnd1_X = %f\n", (double)p->WEnd1_X);
	fprintf(stderr, "RecType2: WEnd1_Y = %f\n", (double)p->WEnd1_Y);
	fprintf(stderr, "RecType2: WEnd1_Z = %f\n", (double)p->WEnd1_Z);
	fprintf(stderr, "RecType2: WEnd2_X = %f\n", (double)p->WEnd2_X);
	fprintf(stderr, "RecType2: WEnd2_Y = %f\n", (double)p->WEnd2_Y);
	fprintf(stderr, "RecType2: WEnd2_Z = %f\n", (double)p->WEnd2_Z);
	fprintf(stderr, "RecType2: WireDia = %f\n", (double)p->WireDia);
	fprintf(stderr, "RecType2: WDiaGa = %d\n", p->WDiaGa);
	fprintf(stderr, "RecType2: WSegs = %d\n", p->WSegs);
	fprintf(stderr, "RecType2: SWNr = %d\n", p->SWNr);
	fprintf(stderr, "RecType2: SWPct = %f\n", (double)p->SWPct);
	fprintf(stderr, "RecType2: SMP_M = %f\n", (double)p->SMP_M);
	fprintf(stderr, "RecType2: SMP_Pdeg = %f\n", (double)p->SMP_Pdeg);
	fprintf(stderr, "RecType2: Stype = %d\n", p->Stype);
	fprintf(stderr, "RecType2: LWNr = %d\n", p->LWNr);
	fprintf(stderr, "RecType2: LWPct = %f\n", (double)p->LWPct);
	fprintf(stderr, "RecType2: LZ_R = %f\n", (double)p->LZ_R);
	fprintf(stderr, "RecType2: LZ_I = %f\n", (double)p->LZ_I);
	fprintf(stderr, "RecType2: LNum_S0 = %f\n", (double)p->LNum_S0);
	fprintf(stderr, "RecType2: LNum_S1 = %f\n", (double)p->LNum_S1);
	fprintf(stderr, "RecType2: LNum_S2 = %f\n", (double)p->LNum_S2);
	fprintf(stderr, "RecType2: LNum_S3 = %f\n", (double)p->LNum_S3);
	fprintf(stderr, "RecType2: LNum_S4 = %f\n", (double)p->LNum_S4);
	fprintf(stderr, "RecType2: LNum_S5 = %f\n", (double)p->LNum_S5);
	fprintf(stderr, "RecType2: LDen_S0 = %f\n", (double)p->LDen_S0);
	fprintf(stderr, "RecType2: LDen_S1 = %f\n", (double)p->LDen_S1);
	fprintf(stderr, "RecType2: LDen_S2 = %f\n", (double)p->LDen_S2);
	fprintf(stderr, "RecType2: LDen_S3 = %f\n", (double)p->LDen_S3);
	fprintf(stderr, "RecType2: LDen_S4 = %f\n", (double)p->LDen_S4);
	fprintf(stderr, "RecType2: LDen_S5 = %f\n", (double)p->LDen_S5);
	fprintf(stderr, "RecType2: TLWNr1 = %d\n", p->TLWNr1);
	fprintf(stderr, "RecType2: TLWPct1 = %f\n", (double)p->TLWPct1);
	fprintf(stderr, "RecType2: TLWNr2 = %d\n", p->TLWNr2);
	fprintf(stderr, "RecType2: TLWPct2 = %f\n", (double)p->TLWPct2);
	fprintf(stderr, "RecType2: TLZ0 = %f\n", (double)p->TLZ0);
	fprintf(stderr, "RecType2: TLLen = %f\n", (double)p->TLLen);
	fprintf(stderr, "RecType2: TLVF = %f\n", (double)p->TLVF);
	fprintf(stderr, "RecType2: LConn = %d\n", p->LConn);
	fprintf(stderr, "RecType2: strRLCType = %d\n", p->strRLCType);
	fprintf(stderr, "RecType2: sngR = %f\n", (double)p->sngR);
	fprintf(stderr, "RecType2: sngL = %f\n", (double)p->sngL);
	fprintf(stderr, "RecType2: sngC = %f\n", (double)p->sngC);
	fprintf(stderr, "RecType2: sngRFreqMHz = %f\n", (double)p->sngRFreqMHz);
	//fprintf(stderr, "RecType2: Reserved[3] = %f\n", (double)p->Reserved[3]);

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpRecType3(RecType3 *p)
{
	if(!Debug_Flag) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump RecType3                                                            @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "RecType3: NFType = %d\n", p->NFType);
	fprintf(stderr, "RecType3: NFCoordType = %d\n", p->NFCoordType);
	fprintf(stderr, "RecType3: NF1Start = %f\n", (double)p->NF1Start);
	fprintf(stderr, "RecType3: NF1Stop = %f\n", (double)p->NF1Stop);
	fprintf(stderr, "RecType3: NF1Step = %f\n", (double)p->NF1Step);
	fprintf(stderr, "RecType3: NF2Start = %f\n", (double)p->NF2Start);
	fprintf(stderr, "RecType3: NF2Stop = %f\n", (double)p->NF2Stop);
	fprintf(stderr, "RecType3: NF2Step = %f\n", (double)p->NF2Step);
	fprintf(stderr, "RecType3: NF3Start = %f\n", (double)p->NF3Start);
	fprintf(stderr, "RecType3: NF3Stop = %f\n", (double)p->NF3Stop);
	fprintf(stderr, "RecType3: NF3Step = %f\n", (double)p->NF3Step);
	//fprintf(stderr, "RecType3: Reserved[132] = %f\n", (double)p->Reserved[132]);

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpBlkHeader(BlkHeader *p)
{
	if(!Debug_Flag) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump BlkHeader                                                           @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "BlkHeader: BlockType = %d\n", p->BlockType);
	fprintf(stderr, "BlkHeader: BlockLen = %d\n", p->BlockLen);
	fprintf(stderr, "BlkHeader: BlockRev = %d\n", p->BlockRev);

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpFreqSweepBlock(BlkHeader *pH, FreqSweepBlk *p)
{
	if(!Debug_Flag) {
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
	fprintf(stderr, "FreqSweepBlk: DataLen = %d\n", p->DataLen);
	fprintf(stderr, "FreqSweepBlk: FIFLen = %d\n", p->FIFLen);
	fprintf(stderr, "FreqSweepBlk: PPFLen = %d\n", p->PPFLen);
	fprintf(stderr, "FreqSweepBlk: SCPFLen = %d\n", p->SCPFLen);
	fprintf(stderr, "FreqSweepBlk: DFLen = %d\n", p->DFLen);
	fprintf(stderr, "FreqSweepBlk: AFLen = %d\n", p->AFLen);
	fprintf(stderr, "FreqSweepBlk: Flags = %d\n", p->Flags);
	fprintf(stderr, "FreqSweepBlk: FStart = %f\n", (double)p->FStart);
	fprintf(stderr, "FreqSweepBlk: FStop = %f\n", (double)p->FStop);
	fprintf(stderr, "FreqSweepBlk: FStep = %f\n", (double)p->FStep);

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

	if(!Debug_Flag) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump Wire Insulation Block                                               @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "WireInsBlock: NumWires = %d\n", p->NumWires);
	for(i = 0; i < p->NumWires; i++) {
		fprintf(stderr, "WireInsBlock: DielC[%d] = %f\n", i + 1, p->Wires[i].DielC);
		fprintf(stderr, "WireInsBlock: Thk[%d] = %f\n", i + 1, p->Wires[i].Thk);
		fprintf(stderr, "WireInsBlock: LTan[%d] = %f\n", i + 1, p->Wires[i].LTan);
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

	if(!Debug_Flag) {
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
		fprintf(stderr, "TLineLossBlock: Loss[%d] = %f\n", i + 1, *pFloat++);
	}
	for(i = 0; i < p->NumLines; i++) {
		fprintf(stderr, "TLineLossBlock: LossFreq[%d] = %f\n", i + 1, *pFloat++);
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpTransformerBlock(BlkHeader *pH, TransformerBlockA *pA, TransformerBlockB *pB, TransformerBlockC *pC, TransformerBlockD *pD)
{
	int i;

	if(!Debug_Flag) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump Transformer Block                                                   @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "TransformerBlock: NX = %d\n", pA->NX);

	for(i = 0; i <= pA->NX; i++) {
		fprintf(stderr, "TransformerBlock: Port1 Tx[%d] Wire = %d\n", i, pB->P1WNr);
		fprintf(stderr, "TransformerBlock: Port2 Tx[%d] Wire = %d\n", i, pB->P2WNr);
		++pB;
	}

	for(i = 0; i <= pA->NX; i++) {
		fprintf(stderr, "TransformerBlock: Port1 Tx[%d] Percent = %f\n", i, pC->P1WPct);
		fprintf(stderr, "TransformerBlock: Port2 Tx[%d] Percent = %f\n", i, pC->P2WPct);
		++pC;
	}

	for(i = 0; i <= pA->NX; i++) {
		fprintf(stderr, "TransformerBlock: Port1 Tx[%d] Z = %f\n", i, pD->P1RelZ);
		if(pD->P2RelZ >= 0) {
			fprintf(stderr, "TransformerBlock: Port2 Tx[%d] Z = %f Norm\n", i, pD->P2RelZ);
		} else {
			fprintf(stderr, "TransformerBlock: Port2 Tx[%d] Z = %f Rev\n", i, -pD->P2RelZ);
		}
		++pD;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

void
dumpLNetBlock(
		BlkHeader *pH,
		LNetBlockA *pA,
		LNetBlockB *pB,
		LNetBlockC *pC,
		LNetBlockD *pD,
		LNetBlockE *pE,
		LNetBlockF *pF,
		LNetBlockG *pG
		)
{
	int i;

	if(!Debug_Flag) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump L-Network Block                                                     @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "\n");

	fprintf(stderr, "LNetBlock: NL = %d\n", pA->NL);

	for(i = 0; i <= pA->NL; i++) {
		fprintf(stderr, "LNetBlock: Port1 Tx[%d] Wire = %d\n", i, pB->P1WNr);
		fprintf(stderr, "LNetBlock: Port2 Tx[%d] Wire = %d\n", i, pB->P2WNr);
		++pB;
	}

	for(i = 0; i <= pA->NL; i++) {
		fprintf(stderr, "LNetBlock: Port1 Tx[%d] Percent = %g\n", i, pC->P1WPct);
		fprintf(stderr, "LNetBlock: Port2 Tx[%d] Percent = %g\n", i, pC->P2WPct);
		++pC;
	}

	for(i = 0; i <= pA->NL; i++) {
		fprintf(stderr, "LNetBlock: Port1 Tx[%d] R = %g\n", i, pD->B1R);
		fprintf(stderr, "LNetBlock: Port1 Tx[%d] X = %g\n", i, pD->B1X);
		fprintf(stderr, "LNetBlock: Port2 Tx[%d] R = %g\n", i, pD->B2R);
		fprintf(stderr, "LNetBlock: Port2 Tx[%d] X = %g\n", i, pD->B2X);
		++pD;
	}

	fprintf(stderr, "LNetBlock: NrRLC = %d\n", pE->NrRLC);

	if(pE->NrRLC) {
		for(i = 0; i <= pA->NL; i++) {
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] R = %g\n", i, pF->B1rlcR);
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] L = %g\n", i, pF->B1rlcL);
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] C = %g\n", i, pF->B1rlcC);
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] F = %g\n", i, pF->B1rlcF);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] R = %g\n", i, pF->B2rlcR);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] L = %g\n", i, pF->B2rlcL);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] C = %g\n", i, pF->B2rlcC);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] F = %g\n", i, pF->B2rlcF);
			++pF;
		}

		for(i = 0; i <= pA->NL; i++) {
			fprintf(stderr, "LNetBlock: Port1 Tx[%d] Type = %d\n", i, pG->B1RLCType);
			fprintf(stderr, "LNetBlock: Port2 Tx[%d] Type = %d\n", i, pG->B2RLCType);
			++pG;
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

	if(!Debug_Flag) {
		return;
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ Dump Virtual Segment Block                                               @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");

	fprintf(stderr, "VWnr = %d\n", p->VWnr);

	// The item at 0 is a dummy, but print it anyway.
	for(i = 0; i <= p->VWnr; i++) {
		fprintf(stderr, "VSegNr[%d] = %d\n", i, p->VSegNr[i]);
	}

	fprintf(stderr, "\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "@@ End Dump                                                                 @@\n");
	fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	fprintf(stderr, "\n");
}

int
mapInput(char *pName)
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
	if(Debug_Flag) fprintf(stderr, "length of file 0x%x\n", gInputFileSize);

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

	dumpRecType1(gPointers.pRec1);
	return 0;
}

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

		dumpRecType2(gPointers.ppRec2[i]);

		// Move to the next block.
		position += PRIMARY_BS;
	}

	return 0;
}

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

void
printCMCE(FILE *pOut)
{
	char buf[TITLE_LEN + 1];
	char *p;
	char *q;
	int i;

	// String has a fixed length and no null terminator.
	memcpy(buf, gPointers.pRec1->Title, TITLE_LEN);
	buf[TITLE_LEN] = 0;

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

void
printWires(FILE *pOut)
{
	int i;
	RecType2 *pRec2;
	double tmp;

	// We might want to handle virtual wires by finding #18 block first.
	for(i = 0; i < gPointers.pRec1->NW; i++) {
		pRec2 = gPointers.ppRec2[i];

		fprintf(pOut, "GW %5d %7d ", i + 1, pRec2->WSegs);

		fprintf(pOut, "%7g ", pRec2->WEnd1_X / gConvert.xyz);
		fprintf(pOut, "%7g ", pRec2->WEnd1_Y / gConvert.xyz);
		fprintf(pOut, "%7g ", pRec2->WEnd1_Z / gConvert.xyz);

		fprintf(pOut, "%7g ", pRec2->WEnd2_X / gConvert.xyz);
		fprintf(pOut, "%7g ", pRec2->WEnd2_Y / gConvert.xyz);
		fprintf(pOut, "%7g ", pRec2->WEnd2_Z / gConvert.xyz);

		if(pRec2->WDiaGa == (uint16_t)-1) {
			// There is only one scale factor in the GS card, so
			// we cannot show wire diameter in different units
			// than wire endpoints.
			//
			// NEC wants the radius, hence the divide-by-2.
			fprintf(pOut, "%7g\n", (pRec2->WireDia / 2.0) / gConvert.xyz);
		} else {
			// Calculate the wire diameter in meters, based on AWG
			// algorithm.  Works for wire GA 0 and thinner.
			//
			// Print it based on the user's scalefactor.
			//
			// NEC wants the radius, hence the divide-by-2.
			tmp = 0.005 * pow(92.0, ((36.0 - pRec2->WDiaGa) / 39.0)) * 0.0254;
			fprintf(pOut, "%7g\n", (tmp / 2.0) / gConvert.xyz);
		}
	}
	fprintf(pOut, "GS %5d %7d %7g\n", 0, 0, gConvert.xyz);
	fprintf(pOut, "GE\n");
}

int
segmentNumber(int segments, double percent)
{
	return (int)round(((segments - 1) * (percent / 100.0)) + 1);
}

int
printExcitation(FILE *pOut)
{
	int i;
	RecType2 *pRec2;
	RecType2 *pWire;
	int type;
	int wireNo;
	int segNo;

	for(i = 0; i < gPointers.pRec1->NSrc; i++) {
		pRec2 = gPointers.ppRec2[i];
		wireNo = pRec2->SWNr;

		if((wireNo > 0) && (wireNo <= gPointers.pRec1->NW)) {
			pWire = gPointers.ppRec2[wireNo - 1];
		} else {
			fprintf(stderr, "Source %d references wire %d, which doesn't exist\n", i + 1, wireNo);
			return -1;
		}

		switch(pRec2->Stype) {
			case 'I':
				type = 6;
				break;
			case 'J': // FIXME - not sure how to handle this yet.
			case 'W': // FIXME - not sure how to handle this yet.
			default:
				type = 0;
				break;
		}

		segNo = segmentNumber(pWire->WSegs, pRec2->SWPct);
		fprintf(pOut, "EX %5d %7d %7d %7d ", type, wireNo, segNo, 0);

		fprintf(pOut, "%7g %7g\n",
				pRec2->SMP_M * cos(pRec2->SMP_Pdeg * PI / 180.0),
				pRec2->SMP_M * sin(pRec2->SMP_Pdeg * PI / 180.0));
	}
}

int
printLoads(FILE *pOut)
{
	int i;
	RecType2 *pRec2;
	RecType2 *pWire;
	int wireNo;
	int segNo;
	int type;

	double v;

	for(i = 0; i < gPointers.pRec1->NL; i++) {
		pRec2 = gPointers.ppRec2[i];
		wireNo = pRec2->LWNr;

		if((wireNo > 0) && (wireNo <= gPointers.pRec1->NW)) {
			pWire = gPointers.ppRec2[wireNo - 1];
		} else {
			fprintf(stderr, "Load %d references wire %d, which doesn't exist\n", i + 1, wireNo);
			return -1;
		}

		segNo = segmentNumber(pWire->WSegs, pRec2->LWPct);

		if(gPointers.pRec1->LType == 'Z') {
			fprintf(pOut, "LD %5d %7d %7d %7d ", 4, wireNo, segNo, segNo);
			fprintf(pOut, "%7g %7g\n", pRec2->LZ_R, pRec2->LZ_I);
		} else if(gPointers.pRec1->LType == 'R') {
			if(pRec2->LConn == 'P') {
				// Parallel
				type = 1;
			} else {
				// Series or unspecified.  Force series to
				// cover the unspecified case.
				pRec2->LConn = 'S';
				type = 0;
			}

			fprintf(pOut, "LD %5d %7d %7d %7d ", type, wireNo, segNo, segNo);
			fprintf(pOut, "%7g %7g %7g\n", pRec2->sngR, pRec2->sngL, pRec2->sngC);
		} else {
			// Laplace
		}
	}
}

void
pass1()
{
}

void
Read_EZNEC(char *InputFile, char *OutputFile)
{
	int			i;
	FILE			*pOut;

	// The following records are all 170 bytes long.
	RecType2		*pRec2;		// Wires, sources, loads, etc
	RecType3		*pRec3;		// Near field data
	//RecType4		*pRec4;		// Reserved block

	// This header comes in front of all variable-length blocks.
	BlkHeader		*pHdr;		// Header for variable length packets

	// Here are the variable-length blocks.
	FreqSweepBlk		*pBlk11;	// File paths and names
	WireInsBlock		*pBlk12;	// Wire insulation
	TLineLossBlock		*pBlk14;	// Transmission Line parameters
	TransformerBlockA	*pBlk15A;	// Transformer parameters - part A
	TransformerBlockB	*pBlk15B;	// Transformer parameters - part B
	TransformerBlockC	*pBlk15C;	// Transformer parameters - part C
	TransformerBlockD	*pBlk15D;	// Transformer parameters - part D
	LNetBlockA		*pBlk17A;	// L Network parameters - part A
	LNetBlockB		*pBlk17B;	// L Network parameters - part B
	LNetBlockC		*pBlk17C;	// L Network parameters - part C
	LNetBlockD		*pBlk17D;	// L Network parameters - part D
	LNetBlockE		*pBlk17E;	// L Network parameters - part E
	LNetBlockF		*pBlk17F;	// L Network parameters - part F
	LNetBlockG		*pBlk17G;	// L Network parameters - part G
	VirtSegmentBlock	*pBlk18;	// Virtual segments

	uint32_t currentPosition;
       	uint32_t BytePosStartBlocks;
	char *mySType[] = { "V", "I", "SV", "SI" };
	char *Config[] = { "Ser", "Par", "Trap"};
	char *ExtConn[] = { "Ser", "Par" };
	int optRjXb;
	uint32_t cboGtypeLI;
	uint32_t cboWireLossLI;
	uint32_t cboGcharLI;
	uint32_t cboPtypeLI;
	char txtPang[STR_LEN];
	uint32_t cboPstepLI;
	int chkMedia2;
	int optMedia2Radial;
	uint32_t cboGchar2LI;

	int WireCnt;
	int SrcOff;
	int MaxedOutSLT;
	int rv;
	int TLOff;
	struct stat sb;
	int XfmrOff;
	int LnetOff;

	uint32_t VSegs[1000];
	uint32_t VCnt;
	uint32_t Vindex;
	int RowOff;

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
	if(mapInput(InputFile) < 0) {
		// Error messages already printed via mapInput.
		exit(1);
	}

	// Map the globals.
	if(mapRecType1() < 0) {
		// Error messages already printed via mapInput.
		exit(1);
	}
	
	// Print the CM-CE header.
	printCMCE(pOut);

	// Set up the conversion factors.  All values in the file use meters,
	// but we want to be able to convert to the user's preferred units.
	setConversionFactors();

	// Map all the RecType2 blocks.
	mapRecType2();

	printWires(pOut);

	printExcitation(pOut);

	printLoads(pOut);

	if(Debug_Flag) fprintf(stderr, "\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "@@ Find Transmission Lines                                                  @@\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "\n");

	if(Debug_Flag) fprintf(stderr, "Number of transmission lines = %d\n", gPointers.pRec1->NT);
	if(gPointers.pRec1->NT > 0) {
		currentPosition = PRIMARY_BS;
		for(TLOff = 1; TLOff <= gPointers.pRec1->NT; TLOff++) {
			MAP(pRec2, RecType2, gIMap, currentPosition);

			dumpRecType2(pRec2);

			if(pRec2->TLWNr1 == (uint16_t)-1) {
				if(Debug_Flag) fprintf(stderr, "End 1 = Short, ");
			} else if(pRec2->TLWNr1 = (uint16_t)-2) {
				if(Debug_Flag) fprintf(stderr, "End 1 = Open, ");
			} else {
				if(Debug_Flag) fprintf(stderr, "End 1 at wire %d, %10.6f %%", pRec2->TLWNr1, pRec2->TLWPct1);
			}

			if(pRec2->TLWNr2 == (uint16_t)-1) {
				if(Debug_Flag) fprintf(stderr, "End 2 = Short, ");
			} else if(pRec2->TLWNr2 = (uint16_t)-2) {
				if(Debug_Flag) fprintf(stderr, "End 2 = Open, ");
			} else {
				if(Debug_Flag) fprintf(stderr, "End 2 at wire %d, %10.6f %%", pRec2->TLWNr2, pRec2->TLWPct2);
			}

			if(pRec2->TLLen > 0) {
				if(Debug_Flag) fprintf(stderr, "TL Length %10.6f, ", pRec2->TLLen / gConvert.xyz);
			} else if(pRec2->TLLen < 0) {
				if(Debug_Flag) fprintf(stderr, "TL Length %10.6f d, ", -pRec2->TLLen);
			} else {
				if(Debug_Flag) fprintf(stderr, "TL Length actual distance, ");
			}

			if(pRec2->TLZ0 >= 0) {
				if(Debug_Flag) fprintf(stderr, "TL Z0 %10.6f N, ", pRec2->TLZ0);
			} else {
				if(Debug_Flag) fprintf(stderr, "TL Z0 %10.6f R, ", -pRec2->TLZ0);
			}
			if(Debug_Flag) fprintf(stderr, "TL VF %10.6f, ", pRec2->TLVF);

			// Move to the next block.
			currentPosition += PRIMARY_BS;
		}
		if(Debug_Flag) fprintf(stderr, "\n");
	}

	if(Debug_Flag) fprintf(stderr, "\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "@@ Find Ground Media                                                        @@\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "\n");

	switch(gPointers.pRec1->Gtype) {
		case 'F':
			cboGtypeLI = 0;
			break;
		case 'P':
			cboGtypeLI = 1;
			break;
		case 'R':
			switch(gPointers.pRec1->GAnal) {
				case 'H':
				case 'F':
					cboGtypeLI = 2;
					break;
				default:
					cboGtypeLI = 3;
			}
	}

	// I think this is trying to find the wire type.  For now, just print the
	// raw values.
	//
	// cboWireLossLI = wsf.Match(wsf.Round(gPointers.pRec1->WRho, 10), .[WireRhos], 0) - 1
	// If Err.Number > 0 Then cboWireLossLI = 5
	// .Range("N6").Value = gPointers.pRec1->WRho
	// .Range("O6").Value = gPointers.pRec1->WMu
	// Rho should be resistivity and Mu should be permeability.  But I'd
	// expect these to be on a load basis rather than global.
	if(Debug_Flag) fprintf(stderr, "WRho %10.6f, ", gPointers.pRec1->WRho);
	if(Debug_Flag) fprintf(stderr, "WMu %10.6f\n", gPointers.pRec1->WMu);

	if(gPointers.pRec1->Gtype == 'R') {

		// Skip over the gPointers.pRec1 block so we can look at the first pRec2 block.
		currentPosition = PRIMARY_BS;
		MAP(pRec2, RecType2, gIMap, currentPosition);

		dumpRecType2(pRec2);

		// cboGcharLI = wsf.Match(wsf.Round(pRec2->MEps + pRec2->MSigma, 4), .[GndCharCombo], 0) - 1
		// If Err.Number > 0 Then cboGcharLI = 11
		// .Range("N4").Value = pRec2->MSigma
		// .Range("O4").Value = pRec2->MEps
		if(Debug_Flag) fprintf(stderr, "MSigma %10.6f, ", pRec2->MSigma);
		if(Debug_Flag) fprintf(stderr, "MEps %10.6f\n", pRec2->MEps);

		if(gPointers.pRec1->NM == 2) {
			chkMedia2 = true;

			// Skip over the gPointers.pRec1 block and the first pRec2 block so we
			// can look at the second pRec2 block.
			currentPosition = PRIMARY_BS + PRIMARY_BS;
			MAP(pRec2, RecType2, gIMap, currentPosition);

			dumpRecType2(pRec2);

			// cboGchar2LI = wsf.Match(wsf.Round(pRec2->MEps + pRec2->MSigma, 4), .[GndCharCombo], 0) - 1
			// If Err.Number > 0 Then
			// cboGchar2LI = 4
			// End If
			// .Range("Q5").Value = RoundSig(CDbl(CStr(pRec2->MHt)) / gConvert.xyz, 7)
			// .Range("R5").Value = RoundSig(CDbl(CStr(pRec2->MCoord)) / gConvert.xyz, 7)
			if(Debug_Flag) fprintf(stderr, "MHt %10.6f, ", pRec2->MHt);
			if(Debug_Flag) fprintf(stderr, "MCoord %10.6f\n", pRec2->MCoord);
			if(gPointers.pRec1->BdryType == 'R') {
			       	optMedia2Radial = true;
			}
		}
	}

	if(gPointers.pRec1->PType == '3') {
		cboPtypeLI = 1;
		snprintf(txtPang, STR_LEN, "%f", 0.0);
	} else if(gPointers.pRec1->PType == 'A' || gPointers.pRec1->OldPType == 'A') {
		cboPtypeLI = 0;
		snprintf(txtPang, STR_LEN, "%f", gPointers.pRec1->Pangle);
	} else if(gPointers.pRec1->PType == 'E' || gPointers.pRec1->OldPType == 'E') {
		cboPtypeLI = 1;
		snprintf(txtPang, STR_LEN, "%f", gPointers.pRec1->Pangle);
	} else {
		cboPtypeLI = 1;
		snprintf(txtPang, STR_LEN, "%f", 0.0);
	}
	if(Debug_Flag) fprintf(stderr, "txtPang %s, ", txtPang);

	if(gPointers.pRec1->PStep == 5) {
		cboPstepLI = 0;
	} else if(gPointers.pRec1->PStep == 1) {
		cboPstepLI = 1;
	} else if(gPointers.pRec1->PStep == 0.5) {
		cboPstepLI = 2;
	} else if(gPointers.pRec1->PStep = 0.1) {
		cboPstepLI = 3;
	} else {
		cboPstepLI = 1;
	}
	if(Debug_Flag) fprintf(stderr, "cboPstepLI %d\n", cboPstepLI);

	if(Debug_Flag) fprintf(stderr, "\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "@@ Find Near Field Parameters                                               @@\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "\n");

	// There is one RecType1, followed by gPointers.pRec1->MaxMWSL RecType2's,
	// then we get to a single RecType3.  In other words, RecType3
	// comes immediately after the last RecType2 record.
	BytePosStartBlocks = PRIMARY_BS + (gPointers.pRec1->MaxMWSL * PRIMARY_BS);
	MAP(pRec3, RecType3, gIMap, currentPosition);
	dumpRecType3(pRec3);

	// There is one RecType4 immediately after the RecType3.  However,
	// It does not currently contain anything, so we skip it.

	if(Debug_Flag) fprintf(stderr, "\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "@@ Start Looking for Var Blocks                                             @@\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "\n");

	// There is always a single gPointers.pRec1 - that's the first PRIMARY_BS in the
	// BytePosStartBlocks variable.
	//
	// Then there are a bunch of pRec2, also PRIMARY_BS bytes each.  NW,
	// NSrc, NL, NT, NM all ride in the same set of pRec2 blocks - that
	// is the parenthesized term.
	//
	// Then comes pRec3 (second to last PRIMARY_BS), and finally comes
	// pRec4 (the last PRIMARY_BS).
	BytePosStartBlocks = PRIMARY_BS + (gPointers.pRec1->MaxMWSL * PRIMARY_BS) + PRIMARY_BS + PRIMARY_BS;

	if(BytePosStartBlocks >= gInputFileSize) {
		// There are no Var Blocks.
		if(Debug_Flag) fprintf(stderr, "No Var Blocks\n");
		return;
	}
	if(Debug_Flag) fprintf(stderr, "Var Blocks start at 0x%x\n", BytePosStartBlocks);

	if(Debug_Flag) fprintf(stderr, "\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "@@ Want Frequency Sweep Block                                               @@\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "\n");

	currentPosition = BytePosStartBlocks;
	MAP(pHdr, BlkHeader, gIMap, currentPosition);

	dumpBlkHeader(pHdr);

	if(pHdr->BlockType == 11) {
		if(Debug_Flag) fprintf(stderr, "\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "@@ Found Frequency Sweep Block                                              @@\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "\n");

		VMAP(pBlk11, FreqSweepBlk, gIMap, currentPosition + sizeof(BlkHeader), pHdr->BlockLen - sizeof(BlkHeader));

		dumpFreqSweepBlock(pHdr, pBlk11);

		if(pBlk11->FStep == 0) {
			// Step size is zero, so there is only a single frequency.
			// We cannot use pBlk11->FStart because it is likely zero
			// also.
			if(Debug_Flag) fprintf(stderr, "TCFreq %10.6f\n", gFrequency);
		} else {
			double Freq;
			int i;

			Freq = pBlk11->FStart;
			for(i = 1; i <= round(((double)pBlk11->FStop - (double)pBlk11->FStart) / (double)pBlk11->FStep) + 1; i++) {
				if(Debug_Flag) fprintf(stderr, "TCFreq %10.6f\n", Freq);
				Freq += pBlk11->FStep;
			}
		}
	} else {
		if(Debug_Flag) fprintf(stderr, "\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "@@ Skip - Not Frequency Sweep Block                                         @@\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "\n");

		// There is no type 11 block, and this is the only place it
		// could be.  Assume we are just looking at a single frequency.
		// FIXME - perhaps it could be elsewhere.  Who says these are in order???
		if(Debug_Flag) fprintf(stderr, "TCFreq %10.6f\n", gFrequency);
	}

	if(Debug_Flag) fprintf(stderr, "\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "@@ Want Wire Insulation Block                                               @@\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "\n");

	currentPosition = BytePosStartBlocks;
	while(currentPosition < gInputFileSize) {
		MAP(pHdr, BlkHeader, gIMap, currentPosition);

		dumpBlkHeader(pHdr);

		if(pHdr->BlockType == 12) {
			int i;

			if(Debug_Flag) fprintf(stderr, "\n");
			if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
			if(Debug_Flag) fprintf(stderr, "@@ Found Wire Insulation Block                                              @@\n");
			if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
			if(Debug_Flag) fprintf(stderr, "\n");

			VMAP(pBlk12, WireInsBlock, gIMap, currentPosition + sizeof(BlkHeader), pHdr->BlockLen - sizeof(BlkHeader));

			dumpWireInsBlock(pHdr, pBlk12);

			for(i = 0; i < pBlk12->NumWires; i++) {
				if(Debug_Flag) fprintf(stderr, "Dielectric C %10.6f, ", pBlk12->Wires[i].DielC);
				if(Debug_Flag) fprintf(stderr, "Thickness %10.6f\n", pBlk12->Wires[i].Thk / gConvert.wdiam);
				if(Debug_Flag) fprintf(stderr, "Loss Tangent %10.6f\n", pBlk12->Wires[i].LTan);
			}
			break;
		} else {
			if(Debug_Flag) fprintf(stderr, "\n");
			if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
			if(Debug_Flag) fprintf(stderr, "@@ Skip - Not Wire Insulation Block                                         @@\n");
			if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
			if(Debug_Flag) fprintf(stderr, "\n");
		}

		currentPosition += pHdr->BlockLen;
	}

	if(gPointers.pRec1->NT > 0) {
		if(Debug_Flag) fprintf(stderr, "\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "@@ Want Transmission Line Loss Parameters                                   @@\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "\n");

		currentPosition = BytePosStartBlocks;
		while(currentPosition < gInputFileSize) {
			uint32_t start;
			uint32_t remain;

			MAP(pHdr, BlkHeader, gIMap, currentPosition);
			start = currentPosition + sizeof(BlkHeader);
			remain = pHdr->BlockLen - sizeof(BlkHeader);

			dumpBlkHeader(pHdr);

			// The type 14 block consists of a pair of variable length arrays.
			// Thus, a structure isn't really appropriate.
			//
			// There are gPointers.pRec1->NT floats representing Loss, followed
			// by gPointers.pRec1->NT floats representing LossFreq.
			if(pHdr->BlockType == 14) {
				int i;
				float *pFloat;

				if(Debug_Flag) fprintf(stderr, "\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "@@ Found Transmission Line Loss Parameters                                  @@\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "\n");

				// Map the block.
				VMAP(pBlk14, TLineLossBlock, gIMap, start, remain);

				dumpTLineLossBlock(pHdr, pBlk14);

				pFloat = &pBlk14->Loss[0];
				for(TLOff = 1; TLOff <= gPointers.pRec1->NT; TLOff++) {
					if(Debug_Flag) fprintf(stderr, "Loss %10.6f\n", *pFloat++ * gConvert.tldB * 100);
				}

				for(TLOff = 1; TLOff <= gPointers.pRec1->NT; TLOff++) {
					if(Debug_Flag) fprintf(stderr, "LossFreq %10.6f\n", *pFloat++);
				}

				TLOff = gPointers.pRec1->NT;
				break;
			} else {
				if(Debug_Flag) fprintf(stderr, "\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "@@ Skip - Not Transmission Line Loss Parameters                             @@\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "\n");
			}

			currentPosition += pHdr->BlockLen;
		}
	}

	if(gPointers.pRec1->NX > 0) {
		if(Debug_Flag) fprintf(stderr, "\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "@@ Transformer Parameters                                                   @@\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "\n");

		currentPosition = BytePosStartBlocks;
		while(currentPosition < gInputFileSize) {
			uint32_t start;
			uint32_t remain;
			uint32_t need;

			MAP(pHdr, BlkHeader, gIMap, currentPosition);
			start = currentPosition + sizeof(BlkHeader);
			remain = pHdr->BlockLen - sizeof(BlkHeader);

			dumpBlkHeader(pHdr);

			if(pHdr->BlockType == 15) {
				if(Debug_Flag) fprintf(stderr, "\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "@@ Found Transformer Parameters                                             @@\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "\n");

				// Map the blocks.
				need = sizeof(TransformerBlockA);
				VMAP(pBlk15A, TransformerBlockA, gIMap, start, need);
				start += need;
				remain -= need;

				// There is one extra (dummy/reserved) block at the start,
				// hence the "1 +".
				need = (1 + pBlk15A->NX) * sizeof(TransformerBlockB);
				VMAP(pBlk15B, TransformerBlockB, gIMap, start, need);
				start += need;
				remain -= need;

				need = (1 + pBlk15A->NX) * sizeof(TransformerBlockC);
				VMAP(pBlk15C, TransformerBlockC, gIMap, start, need);
				start += need;
				remain -= need;

				need = (1 + pBlk15A->NX) * sizeof(TransformerBlockD);
				VMAP(pBlk15D, TransformerBlockD, gIMap, start, need);
				start += need;
				remain -= need;

				dumpTransformerBlock(pHdr, pBlk15A, pBlk15B, pBlk15C, pBlk15D);

				++pBlk15B; // Skip the dummy
				for(XfmrOff = 1; XfmrOff <= pBlk15A->NX; XfmrOff++) {
					if(Debug_Flag) fprintf(stderr, "P1WNr %d, ", pBlk15B->P1WNr);
					if(Debug_Flag) fprintf(stderr, "P2WNr %d\n", pBlk15B->P2WNr);

					++pBlk15B;
				}

				++pBlk15C; // Skip the dummy
				for(XfmrOff = 1; XfmrOff <= pBlk15A->NX; XfmrOff++) {
					if(Debug_Flag) fprintf(stderr, "P1WPct %f, ", pBlk15C->P1WPct);
					if(Debug_Flag) fprintf(stderr, "P2WPct %f\n", pBlk15C->P2WPct);

					++pBlk15C;
				}

				++pBlk15D; // Skip the dummy
				for(XfmrOff = 1; XfmrOff <= pBlk15A->NX; XfmrOff++) {
					if(Debug_Flag) fprintf(stderr, "P1RelZ %f, ", pBlk15D->P1RelZ);
					if(pBlk15D->P2RelZ >= 0) {
						if(Debug_Flag) fprintf(stderr, "P2RelZ %f N\n", pBlk15D->P2RelZ);
					} else {
						if(Debug_Flag) fprintf(stderr, "P2RelZ %f R\n", -pBlk15D->P2RelZ);
					}

					++pBlk15D;
				}
				break;
			} else {
				if(Debug_Flag) fprintf(stderr, "\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "@@ Skip - Not Transformer Parameters                                        @@\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "\n");
			}

			currentPosition += pHdr->BlockLen;
		}
	}

	if(gPointers.pRec1->NLNet > 0) {
		if(Debug_Flag) fprintf(stderr, "\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "@@ LNet Parameters                                                          @@\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "\n");

		currentPosition = BytePosStartBlocks;
		while(currentPosition < gInputFileSize) {
			uint32_t start;
			uint32_t remain;
			uint32_t need;

			MAP(pHdr, BlkHeader, gIMap, currentPosition);
			start = currentPosition + sizeof(BlkHeader);
			remain = pHdr->BlockLen - sizeof(BlkHeader);

			dumpBlkHeader(pHdr);

			if(pHdr->BlockType == 17) {
				if(Debug_Flag) fprintf(stderr, "\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "@@ Found LNet Parameters                                                    @@\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "\n");

				// Map the blocks.
				need = sizeof(LNetBlockA);
				VMAP(pBlk17A, LNetBlockA, gIMap, start, need);
				start += need;
				remain -= need;

				// There is one extra (dummy/reserved) block at the start,
				// hence the "1 +".
				need = (1 + pBlk17A->NL) * sizeof(LNetBlockB);
				VMAP(pBlk17B, LNetBlockB, gIMap, start, need);
				start += need;
				remain -= need;

				// There is one extra (dummy/reserved) block at the start,
				// hence the "1 +".
				need = (1 + pBlk17A->NL) * sizeof(LNetBlockC);
				VMAP(pBlk17C, LNetBlockC, gIMap, start, need);
				start += need;
				remain -= need;

				// There is one extra (dummy/reserved) block at the start,
				// hence the "1 +".
				need = (1 + pBlk17A->NL) * sizeof(LNetBlockD);
				VMAP(pBlk17D, LNetBlockD, gIMap, start, need);
				start += need;
				remain -= need;

				need = sizeof(LNetBlockE);
				VMAP(pBlk17E, LNetBlockE, gIMap, start, need);
				start += need;
				remain -= need;

				if(pBlk17E->NrRLC) {
					// These last two blocks are optional.

					// There is one extra (dummy/reserved) block at the start,
					// hence the "1 +".
					need = (1 + pBlk17A->NL) * sizeof(LNetBlockF);
					VMAP(pBlk17F, LNetBlockF, gIMap, start, need);
					start += need;
					remain -= need;

					// There is one extra (dummy/reserved) block at the start,
					// hence the "1 +".
					need = (1 + pBlk17A->NL) * sizeof(LNetBlockG);
					VMAP(pBlk17G, LNetBlockG, gIMap, start, need);
					start += need;
					remain -= need;
				} else {
					pBlk17F = 0;
					pBlk17G = 0;
				}

				dumpLNetBlock(pHdr, pBlk17A, pBlk17B, pBlk17C, pBlk17D, pBlk17E, pBlk17F, pBlk17G);

				for(LnetOff = 1; LnetOff <= pBlk17A->NL; LnetOff++) {
					if(Debug_Flag) fprintf(stderr, "P1WNr %d, ", pBlk17B[LnetOff].P1WNr);
					if(Debug_Flag) fprintf(stderr, "P2WNr %d\n", pBlk17B[LnetOff].P2WNr);

				}

				for(LnetOff = 1; LnetOff <= pBlk17A->NL; LnetOff++) {
					if(Debug_Flag) fprintf(stderr, "P1WPct %10.6f, ", pBlk17C[LnetOff].P1WPct);
					if(Debug_Flag) fprintf(stderr, "P2WPct %10.6f\n", pBlk17C[LnetOff].P2WPct);

				}

				if(gPointers.pRec1->LNetType == 'Z') {
					for(LnetOff = 1; LnetOff <= pBlk17A->NL; LnetOff++) {
						if(Debug_Flag) fprintf(stderr, "B1R %10.6f, ", pBlk17D[LnetOff].B1R);
						if(Debug_Flag) fprintf(stderr, "B1X %10.6f, ", pBlk17D[LnetOff].B1X);
						if(Debug_Flag) fprintf(stderr, "B2R %10.6f, ", pBlk17D[LnetOff].B2R);
						if(Debug_Flag) fprintf(stderr, "B2X %10.6f\n", pBlk17D[LnetOff].B2X);
					}

					optRjXb = true;
				} else {
					optRjXb = false;

					for(LnetOff = 1; LnetOff <= pBlk17A->NL; LnetOff++) {
						if(Debug_Flag) fprintf(stderr, "B1rlcR %10.6f, ", pBlk17F[LnetOff].B1rlcR);
						if(Debug_Flag) fprintf(stderr, "B1rlcL %10.6f, ", pBlk17F[LnetOff].B1rlcL * 1000000);
						if(Debug_Flag) fprintf(stderr, "B1rlcC %10.6f, ", pBlk17F[LnetOff].B1rlcC * 1000000000000);
						if(Debug_Flag) fprintf(stderr, "B1rlcF %10.6f, ", pBlk17F[LnetOff].B1rlcF);
						if(Debug_Flag) fprintf(stderr, "B2rlcR %10.6f, ", pBlk17F[LnetOff].B2rlcR);
						if(Debug_Flag) fprintf(stderr, "B2rlcL %10.6f, ", pBlk17F[LnetOff].B2rlcL * 1000000);
						if(Debug_Flag) fprintf(stderr, "B2rlcC %10.6f, ", pBlk17F[LnetOff].B2rlcC * 1000000000000);
						if(Debug_Flag) fprintf(stderr, "B2rlcF %10.6f\n", pBlk17F[LnetOff].B2rlcF);

					}

					for(LnetOff = 1; LnetOff <= pBlk17A->NL; LnetOff++) {
						switch(pBlk17G[LnetOff].B1RLCType) {
							case 'S': strlcpy(B1RLCType_Value, Config[0], STR_LEN); break;
							case 'P': strlcpy(B1RLCType_Value, Config[1], STR_LEN); break;
							case 'T': strlcpy(B1RLCType_Value, Config[2], STR_LEN); break;
							default:  strlcpy(B1RLCType_Value, "?", STR_LEN); break;
						}
						switch(pBlk17G[LnetOff].B2RLCType) {
							case 'S': strlcpy(B2RLCType_Value, Config[0], STR_LEN); break;
							case 'P': strlcpy(B2RLCType_Value, Config[1], STR_LEN); break;
							case 'T': strlcpy(B2RLCType_Value, Config[2], STR_LEN); break;
							default:  strlcpy(B2RLCType_Value, "?", STR_LEN); break;
						}
						if(Debug_Flag) fprintf(stderr, "ConfigB1 %s, ConfigB2 %s\n", B1RLCType_Value, B2RLCType_Value);
					}
				}
				break;
			} else {
				if(Debug_Flag) fprintf(stderr, "\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "@@ Skip - Not LNet Parameters                                               @@\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "\n");
			}

			currentPosition += pHdr->BlockLen;
		}
	}

	if(gPointers.pRec1->NVirt > 0) {
		if(Debug_Flag) fprintf(stderr, "\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "@@ Virt Parameters                                                          @@\n");
		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		if(Debug_Flag) fprintf(stderr, "\n");

		currentPosition = BytePosStartBlocks;
		while(currentPosition < gInputFileSize) {
			uint32_t start;
			uint32_t remain;
			uint32_t need;

			MAP(pHdr, BlkHeader, gIMap, currentPosition);
			start = currentPosition + sizeof(BlkHeader);
			remain = pHdr->BlockLen - sizeof(BlkHeader);

			dumpBlkHeader(pHdr);

			if(pHdr->BlockType == 18) {
				if(Debug_Flag) fprintf(stderr, "\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "@@ Found Virt Parameters                                                    @@\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "\n");

				// Map the block.
				VMAP(pBlk18, VirtSegmentBlock, gIMap, currentPosition + sizeof(BlkHeader), pHdr->BlockLen - sizeof(BlkHeader));

				dumpVirtSegmentBlock(pHdr, pBlk18);

				if(Debug_Flag) fprintf(stderr, "VWnr %d\n", pBlk18->VWnr);

				for(VCnt = 1; VCnt <= gPointers.pRec1->NVirt; VCnt++) {
					if(Debug_Flag) fprintf(stderr, "VSegNr %d\n", pBlk18->VSegNr[VCnt]);
				}
				break;
			} else {
				if(Debug_Flag) fprintf(stderr, "\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "@@ Skip - Not Virt Parameters                                               @@\n");
				if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
				if(Debug_Flag) fprintf(stderr, "\n");
			}

			currentPosition += pHdr->BlockLen;
		}

		if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
		fprintf(stderr, "gPointers.pRec1->NSrc %d\n", gPointers.pRec1->NSrc);
		for(RowOff = 1; RowOff <= gPointers.pRec1->NSrc; RowOff++) {
//If .Offset(RowOff, 1) = pBlk18->VWnr Then
//Vindex = wsf.Round((.Offset(RowOff, 2) / 100 + 1 / (gPointers.pRec1->NVirt * 2)) * gPointers.pRec1->NVirt, 0)
//.Offset(RowOff, 1).Value = "V" &amp; CStr(VSegs(Vindex))
//.Offset(RowOff, 2).ClearContents
//End If
		}

		fprintf(stderr, "gPointers.pRec1->NL %d\n", gPointers.pRec1->NL);
		for(RowOff = 1; RowOff <= gPointers.pRec1->NL; RowOff++) {
//If .Offset(RowOff, 1) = pBlk18->VWnr Then
//Vindex = wsf.Round((.Offset(RowOff, 2) / 100 + 1 / (gPointers.pRec1->NVirt * 2)) * gPointers.pRec1->NVirt, 0)
//.Offset(RowOff, 1).Value = "V" &amp; CStr(VSegs(Vindex))
//.Offset(RowOff, 2).ClearContents
//End If
		}

		fprintf(stderr, "gPointers.pRec1->NT %d\n", gPointers.pRec1->NT);
		for(RowOff = 1; RowOff <= gPointers.pRec1->NT; RowOff++) {
//If .Offset(RowOff, 1) = pBlk18->VWnr Then
//Vindex = wsf.Round((.Offset(RowOff, 2) / 100 + 1 / (gPointers.pRec1->NVirt * 2)) * gPointers.pRec1->NVirt, 0)
//.Offset(RowOff, 1).Value = "V" &amp; CStr(VSegs(Vindex))
//.Offset(RowOff, 2).ClearContents
//End If
//If .Offset(RowOff, 3) = pBlk18->VWnr Then
//Vindex = wsf.Round((.Offset(RowOff, 4) / 100 + 1 / (gPointers.pRec1->NVirt * 2)) * gPointers.pRec1->NVirt, 0)
//.Offset(RowOff, 3).Value = "V" &amp; CStr(VSegs(Vindex))
//.Offset(RowOff, 4).ClearContents
//End If
		}

		fprintf(stderr, "gPointers.pRec1->NX %d\n", gPointers.pRec1->NX);
		for(RowOff = 1; RowOff <= gPointers.pRec1->NX; RowOff++) {
//If .Offset(RowOff, 1) = pBlk18->VWnr Then
//Vindex = wsf.Round((.Offset(RowOff, 2) / 100 + 1 / (gPointers.pRec1->NVirt * 2)) * gPointers.pRec1->NVirt, 0)
//.Offset(RowOff, 1).Value = "V" &amp; CStr(VSegs(Vindex))
//.Offset(RowOff, 2).ClearContents
//End If
//If .Offset(RowOff, 3) = pBlk18->VWnr Then
//Vindex = wsf.Round((.Offset(RowOff, 4) / 100 + 1 / (gPointers.pRec1->NVirt * 2)) * gPointers.pRec1->NVirt, 0)
//.Offset(RowOff, 3).Value = "V" &amp; CStr(VSegs(Vindex))
//.Offset(RowOff, 4).ClearContents
//End If
		}

		fprintf(stderr, "gPointers.pRec1->NLNet %d\n", gPointers.pRec1->NLNet);
		for(RowOff = 1; RowOff <= gPointers.pRec1->NLNet; RowOff += 2) {
//If .Offset(RowOff, 1) = pBlk18->VWnr Then
//Vindex = wsf.Round((.Offset(RowOff, 2) / 100 + 1 / (gPointers.pRec1->NVirt * 2)) * gPointers.pRec1->NVirt, 0)
//.Offset(RowOff, 1).Value = "V" &amp; CStr(VSegs(Vindex))
//.Offset(RowOff, 2).ClearContents
//End If
//If .Offset(RowOff + 1, 1) = pBlk18->VWnr Then
//Vindex = wsf.Round((.Offset(RowOff + 1, 2) / 100 + 1 / (gPointers.pRec1->NVirt * 2)) * gPointers.pRec1->NVirt, 0)
//.Offset(RowOff + 1, 1).Value = "V" &amp; CStr(VSegs(Vindex))
//.Offset(RowOff + 1, 2).ClearContents
//End If
		}

//Range(Sheets("Wires").Cells(10 + pBlk18->VWnr, "B"), Sheets("Wires").Cells(10 + pBlk18->VWnr, "I")).ClearContents
	}

	if(Debug_Flag) fprintf(stderr, "\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "@@ Done Looking for Var Blocks                                              @@\n");
	if(Debug_Flag) fprintf(stderr, "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@\n");
	if(Debug_Flag) fprintf(stderr, "\n");

	if(optRjXb) {
		if(Debug_Flag) fprintf(stderr, "RjXb true\n");
	} else {
		if(Debug_Flag) fprintf(stderr, "RLCb true\n");
	}

	if(Debug_Flag) fprintf(stderr, "cboGtype.ListIndex %d\n", cboGtypeLI);
	if(Debug_Flag) fprintf(stderr, "cboWireLoss.ListIndex %d\n", cboWireLossLI);
	if(Debug_Flag) fprintf(stderr, "cboGchar.ListIndex %d\n", cboGcharLI);
	if(Debug_Flag) fprintf(stderr, "cboPtype.ListIndex %d\n", cboPtypeLI);
	if(Debug_Flag) fprintf(stderr, "txtPangle %s\n", txtPang);
	if(Debug_Flag) fprintf(stderr, "cboPstep.ListIndex %d\n", cboPstepLI);

	if(chkMedia2) {
		if(Debug_Flag) fprintf(stderr, "chkMedia2 true\n");
		if(Debug_Flag) fprintf(stderr, "cboGchar2.ListIndex %d\n", cboGchar2LI);

		if(optMedia2Radial) {
			if(Debug_Flag) fprintf(stderr, "optMedia2Radial true\n");
		} else {
			if(Debug_Flag) fprintf(stderr, "optMedia2Linear true\n");
		}
	}

#if 0
	Dim FTOtxt As String
	Dim FromBk As String
	Dim FromSht As Worksheet
	Dim AEwbk As String
	AEwbk = ActiveWorkbook.Name
	Dim c As Range
	Dim k As Integer
	Dim QuitRow As Integer
	FTOtxt = Left(InputFile, Len(InputFile) - 2) & "txt"
	If Len(Dir(FTOtxt)) > 0 Then
		Workbooks.OpenText FileName:=FTOtxt, Origin:=xlWindows, StartRow:=1, DataType:=xlDelimited, TextQualifier:=xlDoubleQuote, ConsecutiveDelimiter:=True, Tab:=False, Semicolon:=False, Comma:=False, Space:=False, Other:=False, FieldInfo:=Array(Array(1, 2))
		FromBk = ActiveWorkbook.Name
		Set FromSht = Workbooks(FromBk).Worksheets(1)
		Workbooks(AEwbk).Activate
		With Sheets("Wires")
			ToRow = ToRow + 1
			QuitRow = ToRow + 200
			.Cells(ToRow, "B").Value = "Antenna Notes:"
			For Each c In Range(FromSht.[A1], FromSht.[A65536].End(xlUp))
				c.Value = wsf.Substitute(c.Value, vbTab, "     ")
				Do
					ToRow = ToRow + 1
					k = InStr(80, c.Value, " ")
					If k > 0 Then
						.Cells(ToRow, "B").Value = "'" & Left(c.Value, k)
						c.Value = Mid(c.Value, k + 1)
					Else
						.Cells(ToRow, "B").Value = c.Value
						c.Value = ""
					End If
				Loop Until Len(c.Value) = 0
				If ToRow = QuitRow Then Exit For
			Next c
		End With
		Workbooks(FromBk).Close SaveChanges:=False
	End If
#endif
}

void
usage(char *prog)
{
	fprintf(stderr, "%s -d -i input_file -o output_file\n", prog);
	fprintf(stderr, "\n");
	fprintf(stderr, "You can provide the input_file with or without the -i\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "If you don't specify an output_file, then output will\n");
	fprintf(stderr, "go to the terminal\n");
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

	while((opt = getopt(argc, argv, "di:o:")) != -1) {
		switch(opt) {
			case 'd':
				Debug_Flag = 1;
				break;

			case 'i':
				pInput = optarg;
				break;

			case 'o':
				pOutput = optarg;
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

	pass1();

	Read_EZNEC(pInput, pOutput);

	exit(0);
}
